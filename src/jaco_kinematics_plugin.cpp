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

/* Author: Sachin Chitta, David Lu!!, Ugo Cupcic */

#include <jaco_kinematics_plugin/jaco_kinematics_plugin.h>
#include <class_loader/class_loader.h>
#include <boost/foreach.hpp>
#include <angles/angles.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread/locks.hpp>
#include <moveit/profiler/profiler.h>


//register KDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(jaco_kinematics_plugin::JacoKinematicsPlugin, kinematics::KinematicsBase)

namespace jaco_kinematics_plugin
{

    JacoKinematicsPlugin::JacoKinematicsPlugin():KDLKinematicsPlugin(),
        empty_dataset(NULL, 0,7)
    {
        initializeConsistencyLimits();
        ros::NodeHandle private_handle("~");
        //joint_state_sub_ = private_handle.subscribe("/joint_states", 50, &JacoKinematicsPlugin::jointStatesCallback, this);
        private_handle.param("use_nearest_neighbors", use_nearest_neighbors_, true);
        private_handle.param("max_neighbor_distance", max_neighbor_distance_, 0.1);
        index_ = new flann::Index< flann::L2<double> >(empty_dataset, flann::KDTreeIndexParams(1));
    }

void JacoKinematicsPlugin::jointStatesCallback(const sensor_msgs::JointState &str) {
    last_state_ = str;
}

void JacoKinematicsPlugin::initializeConsistencyLimits ()
{
    ROS_INFO_STREAM("Initialized Jaco Kinematics plugin");
    consistency_limits_.resize(6, M_PI);
}

bool JacoKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                        const std::vector<double> &ik_seed_state,
                                        std::vector<double> &solution,
                                        moveit_msgs::MoveItErrorCodes &error_code,
                                        const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          default_timeout_,
                          consistency_limits_,
                          solution,
                          solution_callback,
                          error_code,
                          options);
}

bool JacoKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          consistency_limits_,
                          solution,
                          solution_callback,
                          error_code,
                          options);
}



bool JacoKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          consistency_limits_,
                          solution,
                          solution_callback,
                          error_code,
                          options);
}

std::ostream& operator<<(std::ostream& os, const std::vector<double> & obj)
{
    os << "[ ";
    BOOST_FOREACH(const double iter, obj)
    {
        os << iter << ",";
    }
    os <<"]";
  return os;
}

static inline double nearest_equivelent(double desired, double current){
  double previous_rev = floor(current / (2*M_PI));
  double next_rev = ceil(current / (2*M_PI));
  double lower_desired = previous_rev*(2*M_PI) + desired;
  double upper_desired = next_rev*(2*M_PI) + desired;
  if(abs(current - lower_desired) < abs(current - upper_desired))
    return lower_desired;
  return upper_desired;
}

void JacoKinematicsPlugin::cacheLastState(sensor_msgs::JointStateConstPtr &state)
{
    last_state_ = *state;
}

bool JacoKinematicsPlugin::poseToMat(const geometry_msgs::Pose & pose ,flann::Matrix<double> & mat) const
{
    mat[0][0] = pose.position.x;
    mat[0][1] = pose.position.y;
    mat[0][2] = pose.position.z;
    mat[0][3] = pose.orientation.w;
    mat[0][4] = pose.orientation.x;
    mat[0][5] = pose.orientation.y;
    mat[0][6] = pose.orientation.z;
}

bool JacoKinematicsPlugin::matToPose(const flann::Matrix<double> & mat, geometry_msgs::Pose & pose) const
{
    pose.position.x = mat[0][0];
    pose.position.y = mat[0][1];
    pose.position.z = mat[0][2];
    pose.orientation.w = mat[0][3];
    pose.orientation.x = mat[0][4];
    pose.orientation.y = mat[0][5];
    pose.orientation.z = mat[0][6];
}



bool JacoKinematicsPlugin::getNeighbor(const geometry_msgs::Pose & pose, std::vector<int> & query_results) const
{
    boost::mutex::scoped_lock scoped_lock(const_cast<JacoKinematicsPlugin*>(this)->index_mutex_);

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
bool JacoKinematicsPlugin::getClosestTranslation(int & id, const geometry_msgs::Pose & pose, std::vector<double> &translation) const
{
    std::vector<int> query_results;
    if (!getNeighbor(pose, query_results))
        return false;
    id = query_results[0];
    translation = translation_map_.at(id);
    return true;
}

bool JacoKinematicsPlugin::addItem(const geometry_msgs::Pose & pose, const std::vector<double> & translation)
{
   std::vector<double> mat_data(7, 0.0f);
   flann::Matrix<double> mat(&mat_data[0], 1, 7);
   boost::mutex::scoped_lock scoped_lock(index_mutex_);
   poseToMat(pose, mat);
   index_->addPoints(mat, 2);
   ROS_ERROR_STREAM("Added points");
   //There is probably a better way to get the identifier, but for now, do it the easiest way
   int id = index_->size() - 1;
   translation_map_.insert(std::make_pair<int, std::vector<double> >(id, translation));
}

bool JacoKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                              const std::vector<double> &ik_seed_state,
                              double timeout,
                              const std::vector<double> &consistency_limits,
                              std::vector<double> &solution,
                              const IKCallbackFn &solution_callback,
                              moveit_msgs::MoveItErrorCodes &error_code,
                              const kinematics::KinematicsQueryOptions &options) const
{
    moveit::tools::Profiler::ScopedStart prof_start;
    moveit::tools::Profiler::ScopedBlock prof_block("KinematicsPluginLoader::searchPositionIK");
    std::vector <double> desired_joints(6,0);
    int id(-1);
    if (use_nearest_neighbors_)
        getClosestTranslation(id, ik_pose,desired_joints);
    if(id > 0)
   {
        for (int i=0; i < 6; ++i)
            desired_joints[i] = ik_seed_state[i];
    }
    /*BOOST_FOREACH(const double j, ik_seed_state)
        {
           std::cout << "position=" << last_state_.position[i] << std::endl;
           desired_joints[i] = last_state_.position[i];
           ++i;
        }   
        */


    bool success = kdl_kinematics_plugin::KDLKinematicsPlugin::searchPositionIK(ik_pose,
                              desired_joints,
                              timeout,
                              solution,
                              solution_callback,
                              error_code,
                              consistency_limits,
                              options);
    //The wrist positions are equivalent facing either up or down, so we might as well use the closest
//    double rotated_angle_distance;
//    double angle_distance;
//    angles::shortest_angular_distance_with_limits(solution[5]+ M_PI, last_state_.position[5], -2.0*M_PI, 2.0*M_PI, rotated_angle_distance);
//    angles::shortest_angular_distance_with_limits(solution[5], last_state_.position[5], -2.0*M_PI, 2.0*M_PI, angle_distance);
//    if(angle_distance > rotated_angle_distance)
//            solution[5] = solution[5] + M_PI;
//    if(fabs(solution[5] - last_state_.position[5]) > M_PI)
//      solution[5] -= 2.0*M_PI;

    if (success && id < 0 && use_nearest_neighbors_)
        const_cast<JacoKinematicsPlugin*>(this)->addItem(ik_pose, solution);
/*
    ROS_INFO_STREAM("Pose: " << ik_pose << std::endl

                    << " Seed: " << desired_joints << std::endl
                    << " Solution: " << solution << std::endl
                    <<" Limits: " << consistency_limits_);
 */
   moveit::tools::Profiler::Console();
    return success;
}




} // namespace

