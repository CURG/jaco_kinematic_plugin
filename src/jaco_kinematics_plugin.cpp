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

//register KDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(jaco_kinematics_plugin::JacoKinematicsPlugin, kinematics::KinematicsBase)

namespace jaco_kinematics_plugin
{

    JacoKinematicsPlugin::JacoKinematicsPlugin():KDLKinematicsPlugin()
    {
        initializeConsistencyLimits();
        ros::NodeHandle private_handle("~");
        joint_state_sub_ = private_handle.subscribe("/joint_states", 50, &JacoKinematicsPlugin::jointStatesCallback, this);
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

bool JacoKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                              const std::vector<double> &ik_seed_state,
                              double timeout,
                              const std::vector<double> &consistency_limits,
                              std::vector<double> &solution,
                              const IKCallbackFn &solution_callback,
                              moveit_msgs::MoveItErrorCodes &error_code,
                              const kinematics::KinematicsQueryOptions &options) const
{
    std::vector <double> desired_joints(6,0);
    int i = 0;
    for (i=0; i < 6; ++i)
        desired_joints[i] = last_state_.position[i];
    i = 0;
    BOOST_FOREACH(const double j, ik_seed_state)
    {
       std::cout << "last position=" << last_state_.position[i] << std::endl;
       std::cout << "desired position=" << desired_joints[i] << std::endl;
       std::cout << "consistency_limits=" << consistency_limits << std::endl;
       ++i;
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
    ROS_INFO_STREAM("Pose: " << ik_pose << std::endl

                    << " Seed: " << desired_joints << std::endl
                    << " Solution: " << solution << std::endl
                    <<" Limits: " << consistency_limits_);
    return success;
}




} // namespace

