/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.h>
#include <flann/flann.hpp>
#include <ros/ros.h>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <map>

namespace cache_planner_request_adapter
{



class NNCache
{
    typedef double DataType;
    typedef flann::Index<flann::L2<DataType> > IndexType;
    typedef flann::Matrix<DataType> DatasetType;
    typedef boost::shared_ptr<IndexType> IndexPtr;
    typedef boost::shared_ptr<DatasetType> DatasetPtr;    
private:
    size_t feature_len_;
    size_t max_neighbors_;
    boost::shared_mutex index_mutex_;
    IndexPtr index_;
    DatasetPtr dataset_;

protected:

    virtual size_t feature_len(){return feature_len_;}
    virtual size_t max_neighbors(){return max_neighbors_;}
public:

    /*Initializes cache with an empty dataset to begin with*/
    NNCache(size_t feature_len, size_t max_neighbors, size_t trees = 1) : feature_len_(feature_len),
                                                        max_neighbors_(max_neighbors)
    {
        dataset_.reset(new DatasetType(NULL, 0, feature_len_));
        index_.reset(new IndexType(*dataset_, flann::KDTreeIndexParams(trees)));
    }

    bool getNeighbors(DatasetPtr query, std::vector<int> & query_results, float max_neighbor_distance) const
    {
        boost::shared_lock<boost::shared_mutex> read_lock(const_cast<NNCache*>(this)->index_mutex_);
        if (index_->size() > 0)
        {
            std::vector<std::vector<size_t> > query_results_tmp;
            std::vector<std::vector<double> >query_distances;
            flann::SearchParams params(32, 0.0, false);


            index_->radiusSearch(*query, query_results_tmp, query_distances, max_neighbor_distance, params);

            for (size_t i = 0; i < query_results_tmp.size(); ++i)
                query_results.insert(query_results.end(), query_results_tmp[i].begin(), query_results_tmp[i].end());

            return query_results.size() > 0;
        }
        return false;
    }

    bool addNeighbor(DatasetPtr newData, std::vector<size_t> & ids)
    {
      /*Get a writeable lock because we are going to modify the cache datastructures*/
      boost::unique_lock<boost::shared_mutex> write_lock(const_cast<NNCache*>(this)->index_mutex_);
      if (max_neighbors_ > 0 && index_->size() < max_neighbors_)
      {
        size_t old_last_index = index_->size();
        index_->addPoints(*newData, 2);
        size_t new_last_index = index_->size();
        ids.insert(ids.end(), old_last_index, new_last_index);
        ROS_ERROR_STREAM("Added points");
        return true;
      }
      return false;
   }
};

class PlanNNCache : public NNCache
{

    typedef planning_interface::MotionPlanResponse OutputType;
    typedef boost::shared_ptr<OutputType> OutputTypePtr;
    typedef planning_interface::MotionPlanRequest InputType;
    typedef std::map<const size_t, OutputType > OutputMap;

protected:
    OutputMap output_map_;

    bool getKeyData(InputType & input, std::vector<double> & data)
    {
        moveit_msgs::Constraints goal_constraint = input.goal_constraints[0];
        // Convert goal joint constraints to goal joint position vector
        data.resize(goal.joint_constraints.size(), 0);
        for(size_t j = 0; j < goal_constraint.joint_constraints.size(); ++j)
        {
            data[j] = goal_constraint.joint_constraints[j].position;
        }
        return true;
    }

    bool getClosestTranslation(std::vector<size_t> & ids, InputType & input, double max_distance, std::vector<OutputType> & translations)
    {
        std::vector<double> data;
        DatasetPtr query_dataset;
        getKeyData(input, data);
        query_dataset.reset(new DatasetType(data.pointer, 1, data.size()));
        std::vector<size_t> query_results;
        getNeighbors(query_dataset, query_results, max_distance);
        translations.clear();
        translations.reserve(query_results.size());
        for(size_t q = 0; q < query_results.size(); ++ q)
        {
           translations.push_back(output_map[query_results[q]]);
        }
        return query_results.size() > 0;
    }

    bool addTranslatedNeighbor(InputType &input, OutputType & output)
    {
        //Get the key
        std::vector<double> data;
        DatasetPtr input_key;
        getKeyData(input, data);
        input_key.reset(new DatasetType(data.pointer, 1, data.size()));
        std::vector<size_t> inserted_indices;
        addNeighbor(input_key, indices);
        for(size_t i = 0; i < indices.size(); ++i)
        {
            output_map_.insert(std::pair<InputType, OutputType>(input_key[i], output));
        }
        return true;
    }


};

class CacheAdapter : public planning_request_adapter::PlanningRequestAdapter
{


protected:
    typedef std::map<const size_t, planning_interface::MotionPlanResponse > TranslationMap;
    flann::Index<flann::L2<double> > *index_;
    flann::Matrix<double> dataset_;
    TranslationMap translation_map_;
    bool use_nearest_neighbors_;
    double max_neighbor_distance_;
    ros::NodeHandle nh_;


public:
  virtual std::string getDescription() const { return "Adapter that searches for nearby solution and uses it as starting point for further plans"; }


    CacheAdapter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~"),
    dataset_(NULL, 0,6)
    {
        nh_.param("use_nearest_neighbors", use_nearest_neighbors_, true);
        nh_.param("max_neighbor_distance", max_neighbor_distance_, 0.1);

    }

  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest &req,
                            planning_interface::MotionPlanResponse &res,
                            std::vector<std::size_t> &added_path_index) const
  {
    robot_state::RobotState start_state = planning_scene->getCurrentState();
    planning_interface::MotionPlanRequest req2(req);

    bool success = planner(planning_scene, req2, res);
    return success;
  }




};

}

CLASS_LOADER_REGISTER_CLASS(cache_planner_request_adapter::CacheAdapter,
                            planning_request_adapter::PlanningRequestAdapter);
