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
#include <moveit/robot_state/conversions.h>
#include <std_srvs/Empty.h>
#include <boost/range/algorithm.hpp>

namespace cache_planner_request_adapter
{



class NNCache
{
public:
    typedef double DataType;
    typedef flann::Index<flann::L2<DataType> > IndexType;
    typedef flann::Matrix<DataType> DatasetType;
    typedef boost::shared_ptr<IndexType> IndexPtr;
    typedef boost::shared_ptr<DatasetType> DatasetPtr;
    /*Initializes cache with an empty dataset to begin with*/

private:
    size_t feature_len_;
    size_t max_neighbors_;
    boost::shared_mutex index_mutex_;
    IndexPtr index_;
    mutable DatasetPtr dataset_;
    mutable std::vector< boost::shared_ptr<std::vector<double> > > points_;
public:

    NNCache(size_t feature_len, size_t max_neighbors, size_t trees = 1) : feature_len_(feature_len),
                                                        max_neighbors_(max_neighbors)
    {
        dataset_.reset(new DatasetType(NULL, 0, feature_len_));
        index_.reset(new IndexType(*dataset_, flann::KDTreeIndexParams(trees)));        
    }

    bool getNeighbors(DatasetPtr query, std::vector<size_t> & query_results, float max_neighbor_distance) const
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

    bool storeData(DatasetPtr newData, DatasetPtr & copiedData)
    {
        boost::shared_ptr<std::vector<double> > dataPtr;
        dataPtr.reset(new std::vector<double>);
        dataPtr->resize(newData->rows*newData->stride/sizeof(DataType),0);
        memcpy(&((*dataPtr)[0]), (*newData)[0], newData->rows*newData->stride);
        copiedData.reset(new DatasetType(&((*dataPtr)[0]), newData->rows, newData->cols, newData->stride));
        this->points_.push_back(dataPtr);
    }

    bool addNeighbor(DatasetPtr newData, std::vector<size_t> & ids)
    {
      /*Get a writeable lock because we are going to modify the cache datastructures*/
      boost::unique_lock<boost::shared_mutex> write_lock(const_cast<NNCache*>(this)->index_mutex_);
      if (max_neighbors_ > 0 && index_->size() < max_neighbors_)
      {
        size_t old_last_index = index_->size();
        DatasetPtr copiedData;
        storeData(newData, copiedData);
        index_->addPoints(*copiedData, 2);
        size_t new_last_index = index_->size();
        for(size_t i = old_last_index; i < new_last_index; ++i)
            ids.push_back(i);
        ROS_ERROR_STREAM("Added points");
        return true;
      }
      return false;
   }

    virtual size_t feature_len(){return feature_len_;}
    virtual size_t max_neighbors(){return max_neighbors_;}
    virtual size_t cache_size(){return index_->size();}
};

class PlanNNCache : public NNCache
{
public:
    typedef planning_interface::MotionPlanResponse OutputType;
    typedef boost::shared_ptr<OutputType> OutputTypePtr;
    typedef planning_interface::MotionPlanRequest InputType;
    typedef std::map<const size_t, OutputType > OutputMap;


protected:
    mutable OutputMap output_map_;
public:
    PlanNNCache(size_t feature_len, size_t max_neighbors, size_t trees = 1):
        NNCache(feature_len, max_neighbors, trees)
    {}

    bool getKeyData(const InputType & input, std::vector<double> & data) const
    {        
        moveit_msgs::Constraints goal_constraint = input.goal_constraints[0];
        // Convert goal joint constraints to goal joint position vector
        data.resize(goal_constraint.joint_constraints.size(), 0);
        for(size_t j = 0; j < goal_constraint.joint_constraints.size(); ++j)
        {
            data[j] = goal_constraint.joint_constraints[j].position;
        }
        ROS_INFO_STREAM("getKeyData data:" << goal_constraint);
        return true;
    }

    bool getClosestTranslation(std::vector<size_t> & ids, InputType * input, float max_distance, std::vector<OutputType> & translations) const
    {
        std::vector<double> data;
        DatasetPtr query_dataset;
        getKeyData(*input, data);
        query_dataset.reset(new DatasetType(&data[0], 1, data.size()));
        std::vector<size_t> query_results;
        getNeighbors(query_dataset, query_results, max_distance);
        translations.clear();
        translations.reserve(query_results.size());
        for(size_t q = 0; q < query_results.size(); ++ q)
        {
           translations.push_back(output_map_.at(query_results[q]));
        }
        return query_results.size() > 0;
    }

    bool addTranslatedNeighbor(const InputType &input, const OutputType & output)
    {
        //Get the key
        std::vector<double> data;
        DatasetPtr input_key;
        getKeyData(input, data);
        input_key.reset(new DatasetType(&data[0], 1, data.size()));
        std::vector<size_t> inserted_indices;
        addNeighbor(input_key, inserted_indices);
        for(size_t i = 0; i < inserted_indices.size(); ++i)
        {
            output_map_.insert(std::make_pair<size_t, OutputType>(inserted_indices[i], output));
        }
        return true;
    }


};

class CacheAdapter : public planning_request_adapter::PlanningRequestAdapter
{


protected:
    flann::Index<flann::L2<double> > *index_;
    flann::Matrix<double> dataset_;
    mutable bool use_nearest_neighbors_;
    mutable double max_neighbor_distance_;
    mutable ros::NodeHandle nh_;
    mutable boost::shared_ptr<PlanNNCache> cache_;
    mutable ros::ServiceServer update_params_srv_;
    mutable ros::ServiceServer reset_cache_srv_;

public:
  virtual std::string getDescription() const { return "Adapter that searches for nearby solution and uses it as starting point for further plans"; }


    CacheAdapter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~")
    {
       reset_cache_srv_ = nh_.advertiseService("reset_cache", &CacheAdapter::resetCache, this);
       update_params_srv_ = nh_.advertiseService("update_params", &CacheAdapter::updateParams, this);
       std_srvs::Empty::Request req;
       std_srvs::Empty::Response res;
       updateParams(req, res);
       resetCache(req, res);
    }

    bool updateParams(std_srvs::Empty::Request &,
                      std_srvs::Empty::Response & )
    {
        nh_.param("use_nearest_neighbors", use_nearest_neighbors_, true);
        nh_.param("max_neighbor_distance", max_neighbor_distance_, 0.1);
        return true;
    }

    bool resetCache(std_srvs::Empty::Request & ,
                    std_srvs::Empty::Response & )
    {
        cache_.reset();
        return true;
    }

  virtual bool prepareCache(const planning_scene::PlanningSceneConstPtr &planning_scene,
                            const planning_interface::MotionPlanRequest &req) const
    {
        size_t num_joints = req.goal_constraints[0].joint_constraints.size();
        if(cache_== NULL || cache_->feature_len() != num_joints)
        {
            cache_.reset(new PlanNNCache(num_joints, 100));
            ROS_INFO_STREAM("Cache invalid: rebuilding");
        }
        return true;
    }

  virtual bool testResponseValidity(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                    const planning_interface::MotionPlanRequest &req,
                                    const planning_interface::MotionPlanResponse &res) const    
    {        

        bool starting_joint_position = planning_scene->getCurrentStateUpdated(req.start_state)->distance(res.trajectory_->getFirstWayPoint()) < .03;
        if(! starting_joint_position)
            return false;
        bool trajectory_validity = planning_scene->isPathValid(*res.trajectory_, req.group_name);
        return trajectory_validity;
    }

    bool robotStateFromGoalConstraints(const planning_interface::MotionPlanRequest &req, robot_state::RobotStatePtr & state) const
    {
        state.reset(new robot_state::RobotState(req.start_state));
        moveit_msgs::Constraints goal_constraint = req.goal_constraints[0];
        for(size_t j = 0; j <goal_constraint.joint_constraints.size(); ++j)
        {
            const moveit_msgs::JointConstraint joint_constraint = goal_constraint.joint_constraints[j];
            state->setVariablePosition(joint_constraint.joint_name, joint_constraint.position);
        }
        return true;
    }

  virtual bool getClosestState(robot_trajectory::RobotTrajectoryPtr & traj, const planning_interface::MotionPlanRequest &req, int & minElementInd) const
    {
        robot_state::RobotStatePtr goal_state;
        if(!robotStateFromGoalConstraints(req, goal_state))
            return false;
        minElementInd = -1;
        double minValue = -1.0f;
        for(size_t w = 0; w < traj->getWayPointCount(); ++w)
        {
            dist = traj->getWayPoint(w).distance(goal_state) < minElement;
            if(dist < minValue)
            {
                minElementInd = w;
                minValue = dist;
            }
        }

        if(minElement < 0)
            return false;
        return true;
    }

  virtual bool modifyRequest(const planning_scene::PlanningSceneConstPtr& planning_scene,
                             const planning_interface::MotionPlanRequest &req,
                             planning_interface::MotionPlanRequest &modified_req,
                             planning_interface::MotionPlanResponse &cached_res_copy) const
    {
        planning_interface::MotionPlanResponse res;
        std::vector<size_t> ids;
        std::vector<PlanNNCache::OutputType> responses;
        modified_req = req;
        if(!cache_->getClosestTranslation(ids, static_cast<PlanNNCache::InputType *>(&modified_req), max_neighbor_distance_, responses))
            return false;
        for(size_t i=0; i < responses.size(); ++i)
        {
            if(testResponseValidity(planning_scene, req, responses[i]))
            {
                res = static_cast<planning_interface::MotionPlanResponse>(responses[i]);
                cached_res_copy.trajectory_.reset(new robot_trajectory::RobotTrajectory(res.trajectory_->getRobotModel(), res.trajectory_->getGroupName()));
                cached_res_copy.trajectory_->append(*res.trajectory_, 0);                
                robot_state::robotStateToRobotStateMsg(cached_res_copy.trajectory_->getLastWayPoint(), modified_req.start_state);
                ROS_INFO_STREAM("Found valid cached plan");
                return true;                
            }
            else
            {
                ROS_INFO_STREAM("Cached plan invalid");
            }
        }

        return false;
    }

  virtual bool appendTrajectory(planning_interface::MotionPlanResponse &cached_res_copy,
                                planning_interface::MotionPlanResponse &new_res) const
  {
        cached_res_copy.trajectory_->append(*new_res.trajectory_, .001);
        cached_res_copy.planning_time_ = new_res.planning_time_;
        cached_res_copy.error_code_ = new_res.error_code_;
        return true;
  }

  virtual bool addToCache(const planning_interface::MotionPlanRequest &req,
                          const planning_interface::MotionPlanResponse &res) const
    {
        return cache_->addTranslatedNeighbor(req, res);
    }

  virtual bool adaptAndPlan(const PlannerFn &planner,
                            const planning_scene::PlanningSceneConstPtr& planning_scene,
                            const planning_interface::MotionPlanRequest &req,
                            planning_interface::MotionPlanResponse &res,
                            std::vector<std::size_t> &added_path_index) const
  {

    ROS_INFO_STREAM("cache_planning_request:: In adaptAndPlan");
    if(!prepareCache(planning_scene, req))
        return false;

    planning_interface::MotionPlanRequest modified_req(req);
    planning_interface::MotionPlanResponse temp_response;
    bool has_modified_request(false);
    if(use_nearest_neighbors_ && modifyRequest(planning_scene, req, modified_req, res))
    {
        has_modified_request = true;
        ROS_INFO_STREAM("Request modified");
    }

    bool success = planner(planning_scene, modified_req, temp_response);
    if(has_modified_request && appendTrajectory(res, temp_response))
    {
        size_t num_added_points = res.trajectory_->getWayPointCount() -
                                temp_response.trajectory_->getWayPointCount();

        for(std::vector<std::size_t>::iterator i = added_path_index.begin(); i != added_path_index.end(); ++i)
            *i += num_added_points;

        added_path_index.reserve(res.trajectory_->getWayPointCount());
        added_path_index.insert(added_path_index.end(), 0, num_added_points);
        ROS_INFO_STREAM("Waypoints added");
    }
    else
    {
        res = temp_response;
    }

    if(use_nearest_neighbors_ && !has_modified_request && success)
    {
        addToCache(req, res);
        ROS_INFO_STREAM("Added to cache. Cache size:" << cache_->cache_size());
    }

    return success;
  }
};

}

CLASS_LOADER_REGISTER_CLASS(cache_planner_request_adapter::CacheAdapter,
                            planning_request_adapter::PlanningRequestAdapter);
