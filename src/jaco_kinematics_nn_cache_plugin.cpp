/*********************************************************************
- * Software License Agreement (BSD License)
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

/* Author: Sachin Chitta, David Lu!!, Ugo Cupcic */

#include <jaco_kinematics_plugin/jaco_kinematics_plugin.h>
#include <class_loader/class_loader.h>
#include <boost/foreach.hpp>
#include <angles/angles.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <moveit/profiler/profiler.h>
#include <flann/flann.hpp>

namespace jaco_kinematics_plugin
{
  /* @brief A kinematics plugin that just adds timing information about the main IK function
   *  
   * This class was constructed just to provide the best possible baseline.
   */
    class JacoKinematicsTimingPlugin: public kdl_kinematics_plugin::KDLKinematicsPlugin
    {

        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      const std::vector<double> &consistency_limits,
                                      std::vector<double> &solution,
                                      const IKCallbackFn &solution_callback,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options) const
        {
            moveit::tools::Profiler::ScopedStart prof_start;
            moveit::tools::Profiler::ScopedBlock prof_block("JacoKinematicsPluginDefault::searchPositionIK");

            bool success = kdl_kinematics_plugin::KDLKinematicsPlugin::searchPositionIK(ik_pose,
                                                                                        ik_seed_state,
                                                                                        timeout,
                                                                                        solution,
                                                                                        solution_callback,
                                                                                        error_code,
                                                                                        consistency_limits,
                                                                                        options);
            moveit::tools::Profiler::Console();
            return success;
        }
    };


    // Register the plugin
    CLASS_LOADER_REGISTER_CLASS(jaco_kinematics_plugin::JacoKinematicsTimingPlugin, kinematics::KinematicsBase)



    /*@brief A class that caches the kinematics requests using the fast approximate nearest neighbors library (FLANN)
     *  This implentation violates the constness that the kinematics plugin tries to enforce. It should be thread safe because access to the cache data structures is handled by the read/write boost::shared_mutex construct described here:
     */
    class JacoKinematicsNNCachePlugin: public kdl_kinematics_plugin::KDLKinematicsPlugin
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

        class KinematicsNNCache : public NNCache {
        public:
            typedef std::vector<double> OutputType;
            typedef boost::shared_ptr<OutputType> OutputTypePtr;
            typedef geometry_msgs::Pose InputType;
            typedef std::map<const size_t, OutputType> OutputMap;


        protected:
            mutable OutputMap output_map_;
        public:
            KinematicsNNCache(size_t feature_len, size_t max_neighbors, size_t trees = 1) :
                    NNCache(feature_len, max_neighbors, trees) {
            }

            bool getKeyData(const InputType &input, std::vector<double> &data) const {
                data.resize(7);
                data[0] = input.position.x;
                data[1] = input.position.y;
                data[2] = input.position.z;
                data[3] = input.orientation.w;
                data[4] = input.orientation.x;
                data[5] = input.orientation.y;
                data[6] = input.orientation.z;
                return true;
            }

            bool getClosestTranslation(std::vector<size_t> &ids, const InputType *input, float max_distance, std::vector<OutputType> &translations) const {
                std::vector<double> data;
                DatasetPtr query_dataset;
                getKeyData(*input, data);
                query_dataset.reset(new DatasetType(&data[0], 1, data.size()));
                std::vector<size_t> query_results;
                getNeighbors(query_dataset, query_results, max_distance);
                translations.clear();
                translations.reserve(query_results.size());
                for (size_t q = 0; q < query_results.size(); ++q) {
                    translations.push_back(output_map_.at(query_results[q]));
                }
                return query_results.size() > 0;
            }

            bool addTranslatedNeighbor(const InputType &input, const OutputType &output) {
                //Get the key
                std::vector<double> data;
                DatasetPtr input_key;
                getKeyData(input, data);
                input_key.reset(new DatasetType(&data[0], 1, data.size()));
                std::vector<size_t> inserted_indices;
                addNeighbor(input_key, inserted_indices);
                for (size_t i = 0; i < inserted_indices.size(); ++i) {
                    output_map_.insert(std::make_pair<size_t, OutputType>(inserted_indices[i], output));
                }
                return true;
            }

        };

        bool use_nearest_neighbors_;
        double max_neighbor_distance_;
        mutable boost::shared_ptr<KinematicsNNCache> cache_;

    public:
        JacoKinematicsNNCachePlugin():KDLKinematicsPlugin()
        {
            ros::NodeHandle private_handle("~");
            private_handle.param("use_nearest_neighbors", use_nearest_neighbors_, true);
            private_handle.param("max_neighbor_distance", max_neighbor_distance_, 0.1);
        }


        virtual bool prepareCache() const
        {
            if(cache_== NULL)
            {
                // geometry_msgs.Pose always has 7 members
                size_t feature_len = 7;
                cache_.reset(new KinematicsNNCache(7, 100));
                ROS_INFO_STREAM("Cache invalid: rebuilding");
            }
            return true;
        }


        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      const std::vector<double> &consistency_limits,
                                      std::vector<double> &solution,
                                      const IKCallbackFn &solution_callback,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options) const
        {
            moveit::tools::Profiler::ScopedStart prof_start;
            moveit::tools::Profiler::ScopedBlock prof_block("JacoKinematicsPluginCachePlugin::searchPositionIK");
            std::vector <double> desired_joints(6,0);
            bool neighbor_found = false;
            if (use_nearest_neighbors_) {
                prepareCache();
                std::vector<size_t> ids;
                std::vector<KinematicsNNCache::OutputType> responses;

                cache_->getClosestTranslation(ids, static_cast<const KinematicsNNCache::InputType *>(&ik_pose), max_neighbor_distance_, responses);
                if(ids.size() > 0) {

                    desired_joints = responses[0];
                    neighbor_found = true;
                }
            }

            if(!neighbor_found)
           {
                for (int i=0; i < 6; ++i)
                    desired_joints[i] = ik_seed_state[i];
            }


            bool success = kdl_kinematics_plugin::KDLKinematicsPlugin::searchPositionIK(ik_pose,
                                                                                        desired_joints,
                                                                                        timeout,
                                                                                        solution,
                                                                                        solution_callback,
                                                                                        error_code,
                                                                                        consistency_limits,
                                                                                        options);
            //while(solution[5] > M_PI)
            //    solution[5] -= 2*M_PI;
            //while(solution[5] < -M_PI)
            //    solution[5] += 2*M_PI;

            //solution[5] += M_PI;

            if (success && !neighbor_found && use_nearest_neighbors_)
                cache_->addTranslatedNeighbor(ik_pose, solution);
	    if(!success && neighbor_found)
	      ROS_ERROR_STREAM("Neighbor found but ik search failed");
            moveit::tools::Profiler::Console();
            return success;
        }

    };
  
} // namespace


//register KDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(jaco_kinematics_plugin::JacoKinematicsNNCachePlugin, kinematics::KinematicsBase)
