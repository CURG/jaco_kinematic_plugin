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

#ifndef JACO_KINEMATICS_PLUGIN_
#define JACO_KINEMATICS_PLUGIN_ 

#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>

namespace jaco_kinematics_plugin
{
/**
 * @brief Specific implementation of kinematics using KDL. This version can be used with any robot.
 */
  class JacoKinematicsPlugin : public kdl_kinematics_plugin::KDLKinematicsPlugin
  {
    public:

    /**
     *  @brief Default constructor
     */
    JacoKinematicsPlugin();

    virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                               const std::vector<double> &ik_seed_state,
                               std::vector<double> &solution,
                               moveit_msgs::MoveItErrorCodes &error_code,
                               const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;



    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;


    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;


  protected:
    /**
     * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
     * This particular method is intended for "searching" for a solutions by stepping through the redundancy
     * (or other numerical routines).
     * @param ik_pose the desired pose of the link
     * @param ik_seed_state an initial guess solution for the inverse kinematics
     * @param timeout The amount of time (in seconds) available to the solver
     * @param solution the solution vector
     * @param solution_callback A callback solution for the IK solution
     * @param error_code an error code that encodes the reason for failure or success
     * @param check_consistency Set to true if consistency check needs to be performed
     * @param redundancy The index of the redundant joint
     * @param consistency_limit The returned solutuion will contain a value for the redundant joint in the range [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
     * @return True if a valid solution was found, false otherwise
     */

//    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
//                          const std::vector<double> &ik_seed_state,
//                          double timeout,
//                          std::vector<double> &solution,
//                          const IKCallbackFn &solution_callback,
//                          moveit_msgs::MoveItErrorCodes &error_code,
//                          const std::vector<double> &consistency_limits,
//                          const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;



//    virtual bool initialize(const std::string &robot_description,
//                            const std::string &group_name,
//                            const std::string &base_name,
//                            const std::string &tip_name,
//                            double search_discretization);

    void initializeConsistencyLimits();
    void jointStatesCallback(const sensor_msgs::JointState &str);


  private:
    //Create a joint states subscriber that caches last state
    ros::Subscriber joint_state_sub_;
    sensor_msgs::JointState last_state_;
    void cacheLastState(sensor_msgs::JointStateConstPtr & state);

    std::vector<double> consistency_limits_;

  };
}

#endif
