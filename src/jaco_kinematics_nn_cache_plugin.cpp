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
#include <boost/thread/shared_lock_guard.hpp>
#include <moveit/profiler/profiler.h>


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



    private:
        typedef std::map<const size_t, std::vector<double> > TranslationMap;

        flann::Index<flann::L2<double> > *index_;
        flann::Matrix<double> empty_dataset;

        TranslationMap translation_map_;
        std::vector<double> consistency_limits_;

        bool poseToMat(const geometry_msgs::Pose & pose ,flann::Matrix<double> & mat) const;
        bool matToPose(const flann::Matrix<double> & mat, geometry_msgs::Pose & pose) const;
        bool getNeighbor(const geometry_msgs::Pose & pose, std::vector<int> & query_results) const;
        bool addItem(const geometry_msgs::Pose & pose, const std::vector<double> & translation);
        bool getClosestTranslation( int & id, const geometry_msgs::Pose & pose, std::vector<double> &translation) const;
        bool use_nearest_neighbors_;
        double max_neighbor_distance_;
        boost::shared_mutex index_mutex_;

    public:
        JacoKinematicsNNCachePlugin():KDLKinematicsPlugin(),
            empty_dataset(NULL, 0,7)
        {
            ros::NodeHandle private_handle("~");
            private_handle.param("use_nearest_neighbors", use_nearest_neighbors_, true);
            private_handle.param("max_neighbor_distance", max_neighbor_distance_, 0.1);
            index_ = new flann::Index< flann::L2<double> >(empty_dataset, flann::KDTreeIndexParams(1));
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
            moveit::tools::Profiler::ScopedBlock prof_block("JacoKinematicsPluginDefault::searchPositionIK");
            std::vector <double> desired_joints(6,0);
            int id(-1);
            if (use_nearest_neighbors_)
                getClosestTranslation(id, ik_pose,desired_joints);
            if(id > 0)
           {
                for (int i=0; i < 6; ++i)
                    desired_joints[i] = ik_seed_state[i];
            }

            bool success = kdl_kinematics_plugin::KDLKinematicsPlugin::searchPositionIK(ik_pose,
                                                                                        ik_seed_state,
                                                                                        timeout,
                                                                                        solution,
                                                                                        solution_callback,
                                                                                        error_code,
                                                                                        consistency_limits,
                                                                                        options);
            if (success && id < 0 && use_nearest_neighbors_)
                const_cast<JacoKinematicsNNCachePlugin*>(this)->addItem(ik_pose, solution);
            moveit::tools::Profiler::Console();
            return success;
        }

    };



    bool JacoKinematicsNNCachePlugin::poseToMat(const geometry_msgs::Pose & pose ,flann::Matrix<double> & mat) const
    {
        mat[0][0] = pose.position.x;
        mat[0][1] = pose.position.y;
        mat[0][2] = pose.position.z;
        mat[0][3] = pose.orientation.w;
        mat[0][4] = pose.orientation.x;
        mat[0][5] = pose.orientation.y;
        mat[0][6] = pose.orientation.z;
    }

    bool JacoKinematicsNNCachePlugin::matToPose(const flann::Matrix<double> & mat, geometry_msgs::Pose & pose) const
    {
        pose.position.x = mat[0][0];
        pose.position.y = mat[0][1];
        pose.position.z = mat[0][2];
        pose.orientation.w = mat[0][3];
        pose.orientation.x = mat[0][4];
        pose.orientation.y = mat[0][5];
        pose.orientation.z = mat[0][6];
    }



    bool JacoKinematicsNNCachePlugin::getNeighbor(const geometry_msgs::Pose & pose, std::vector<int> & query_results) const
    {
      //Read lock to read FLANN index 
      boost::shared_lock<boost::shared_mutex> read_lock(const_cast<JacoKinematicsNNCachePlugin*>(this)->index_mutex_);

        if (index_->size() > 0)
        {
            std::vector<double> query_pose_mat_data(7,0.0f);
            std::vector<std::vector<size_t> > query_results_tmp;
            flann::Matrix<double> query_pose_mat(&query_pose_mat_data[0], 1,7);
            if(!poseToMat(pose ,query_pose_mat))
                return false;
            std::vector<std::vector<double> >query_distances;
            flann::SearchParams params(32, 0.0, false);


            index_->radiusSearch(query_pose_mat, query_results_tmp, query_distances, max_neighbor_distance_, params);

            for (size_t i = 0; i < query_results_tmp.size(); ++i)
                query_results.insert(query_results.end(), query_results_tmp[i].begin(), query_results_tmp[i].end());

            return query_results.size() > 0;
        }
        return false;
    }

    /*A validation function would go here if there was one */
    bool JacoKinematicsNNCachePlugin::getClosestTranslation(int & id, const geometry_msgs::Pose & pose, std::vector<double> &translation) const
    {
      //Read map to access translation map
       boost::shared_lock<boost::shared_mutex> read_lock(const_cast<JacoKinematicsNNCachePlugin*>(this)->index_mutex_);
        std::vector<int> query_results;
        if (!getNeighbor(pose, query_results))
            return false;
        id = query_results[0];
        translation = translation_map_.at(id);
        return true;
    }

    bool JacoKinematicsNNCachePlugin::addItem(const geometry_msgs::Pose & pose, const std::vector<double> & translation)
    {
      /*Get a writeable lock because we are going to modify the cache datastructures*/
      boost::unique_lock<boost::shared_mutex> write_lock(const_cast<JacoKinematicsNNCachePlugin*>(this)->index_mutex_);


       
      std::vector<double> mat_data(7, 0.0f);
      flann::Matrix<double> mat(&mat_data[0], 1, 7);
      poseToMat(pose, mat);
      index_->addPoints(mat, 2);
      ROS_ERROR_STREAM("Added points");
      //There is probably a better way to get the identifier, but for now, do it the easiest way
      int id = index_->size() - 1;
      translation_map_.insert(std::make_pair<int, std::vector<double> >(id, translation));
    }
  
  
} // namespace


//register KDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(jaco_kinematics_plugin::JacoKinematicsNNCachePlugin, kinematics::KinematicsBase)
