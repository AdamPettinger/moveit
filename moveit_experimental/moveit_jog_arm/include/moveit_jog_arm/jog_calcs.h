/*******************************************************************************
 *      Title     : jog_calcs.h
 *      Project   : moveit_jog_arm
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#pragma once

// System
#include <mutex>

// ROS
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ChangeDriftDimensions.h>
#include <moveit_msgs/ChangeControlDimensions.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

// moveit_jog_arm
#include <moveit_jog_arm/jog_arm_parameters.h>
#include <moveit_jog_arm/low_pass_filter.h>
#include <moveit_jog_arm/status_codes.h>
#include <moveit_jog_arm/low_pass_filter.h>
#include <moveit_jog_arm/joint_state_subscriber.h>

namespace moveit_jog_arm
{
class JogCalcs
{
public:
  JogCalcs(ros::NodeHandle& nh, const JogArmParameters& parameters,
           const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
           const std::shared_ptr<JointStateSubscriber>& joint_state_subscriber);

  /** \brief Start and stop the timer where we do work and publish outputs */
  void start();
  void stop();

  /**
   * Get the MoveIt planning link transform.
   * The transform from the MoveIt planning frame to robot_link_command_frame
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getCommandFrameTransform(Eigen::Isometry3d& transform);

  /** \brief Pause or unpause processing jog commands while keeping the timers alive */
  void setPaused(const bool paused);

private:
  /** \brief Timer method */
  void run(const ros::TimerEvent& timer_event);

  /** \brief Do jogging calculations for Cartesian twist commands. */
  bool cartesianJogCalcs(geometry_msgs::TwistStamped& cmd, trajectory_msgs::JointTrajectory& joint_trajectory);

  /** \brief Do jogging calculations for direct commands to a joint. */
  bool jointJogCalcs(const control_msgs::JointJog& cmd, trajectory_msgs::JointTrajectory& joint_trajectory);

  /** \brief Parse the incoming joint msg for the joints of our MoveGroup */
  bool updateJoints();

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   */
  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::TwistStamped& command) const;

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   */
  Eigen::VectorXd scaleJointCommand(const control_msgs::JointJog& command) const;

  bool addJointIncrements(sensor_msgs::JointState& output, const Eigen::VectorXd& increments) const;

  /** \brief Suddenly halt for a joint limit or other critical issue.
   * Is handled differently for position vs. velocity control.
   */
  void suddenHalt(trajectory_msgs::JointTrajectory& joint_trajectory);

  /** \brief  Scale the delta theta to match joint velocity/acceleration limits */
  void enforceSRDFAccelVelLimits(Eigen::ArrayXd& delta_theta);

  /** \brief Avoid overshooting joint limits */
  bool enforceSRDFPositionLimits();

  /** \brief Possibly calculate a velocity scaling factor, due to proximity of
   * singularity and direction of motion
   */
  double velocityScalingFactorForSingularity(const Eigen::VectorXd& commanded_velocity,
                                             const Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                                             const Eigen::MatrixXd& jacobian, const Eigen::MatrixXd& pseudo_inverse);

  /**
   * Slow motion down if close to singularity or collision.
   * @param delta_theta motion command, used in calculating new_joint_tray
   * @param singularity_scale tells how close we are to a singularity
   */
  void applyVelocityScaling(Eigen::ArrayXd& delta_theta, const double singularity_scale);

  /** \brief Compose the outgoing JointTrajectory message */
  void composeJointTrajMessage(const sensor_msgs::JointState& joint_state,
                               trajectory_msgs::JointTrajectory& joint_trajectory) const;

  /** \brief Smooth position commands with a lowpass filter */
  void lowPassFilterPositions(sensor_msgs::JointState& joint_state);

  /** \brief Set the filters to the specified values */
  void resetLowPassFilters(const sensor_msgs::JointState& joint_state);

  /** \brief Convert an incremental position command to joint velocity message */
  void calculateJointVelocities(sensor_msgs::JointState& joint_state, const Eigen::ArrayXd& delta_theta);

  /** \brief Convert joint deltas to an outgoing JointTrajectory command.
    * This happens for joint commands and Cartesian commands.
    */
  bool convertDeltasToOutgoingCmd(trajectory_msgs::JointTrajectory& joint_trajectory);

  /** \brief Gazebo simulations have very strict message timestamp requirements.
   * Satisfy Gazebo by stuffing multiple messages into one.
   */
  void insertRedundantPointsIntoTrajectory(trajectory_msgs::JointTrajectory& joint_trajectory, int count) const;

  /**
   * Remove the Jacobian row and the delta-x element of one Cartesian dimension, to take advantage of task redundancy
   *
   * @param matrix The Jacobian matrix.
   * @param delta_x Vector of Cartesian delta commands, should be the same size as matrix.rows()
   * @param row_to_remove Dimension that will be allowed to drift, e.g. row_to_remove = 2 allows z-translation drift.
   */
  void removeDimension(Eigen::MatrixXd& matrix, Eigen::VectorXd& delta_x, const unsigned int row_to_remove);

  /* \brief Callback for joint subsription */
  void jointStateCB(const sensor_msgs::JointStateConstPtr& msg);

  /* \brief Command callbacks */
  void twistStampedCB(const geometry_msgs::TwistStampedConstPtr& msg);
  void jointJogCB(const control_msgs::JointJogConstPtr& msg);
  void collisionVelocityScaleCB(const std_msgs::Float64ConstPtr& msg);

  /**
   * Allow drift in certain dimensions. For example, may allow the wrist to rotate freely.
   * This can help avoid singularities.
   *
   * @param request the service request
   * @param response the service response
   * @return true if the adjustment was made
   */
  bool changeDriftDimensions(moveit_msgs::ChangeDriftDimensions::Request& req,
                             moveit_msgs::ChangeDriftDimensions::Response& res);

  /** \brief Start the main calculation timer */
  // Service callback for changing jogging dimensions
  bool changeControlDimensions(moveit_msgs::ChangeControlDimensions::Request& req,
                               moveit_msgs::ChangeControlDimensions::Response& res);

  ros::NodeHandle nh_;

  // Parameters from yaml
  const JogArmParameters& parameters_;

  // Pointer to the collision environment
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Subscriber to the latest joint states
  const std::shared_ptr<JointStateSubscriber> joint_state_subscriber_;

  // Track the number of cycles during which motion has not occurred.
  // Will avoid re-publishing zero velocities endlessly.
  int zero_velocity_count_ = 0;

  // Flag for staying inactive while there are no incoming commands
  bool wait_for_jog_commands_ = true;

  // Flag saying if the filters were updated during the timer callback
  bool updated_filters_ = false;

  // Nonzero status flags
  bool have_nonzero_twist_stamped_ = false;
  bool have_nonzero_joint_jog_ = false;
  bool have_nonzero_command_ = false;

  // Incoming command messages
  geometry_msgs::TwistStamped twist_stamped_;
  control_msgs::JointJog joint_jog_;

  const moveit::core::JointModelGroup* joint_model_group_;

  moveit::core::RobotStatePtr kinematic_state_;

  // incoming_joint_state_ is the incoming message. It may contain passive joints or other joints we don't care about.
  // (mutex protected below)
  // internal_joint_state_ is used in jog calculations. It shouldn't be relied on to be accurate.
  // original_joint_state_ is the same as incoming_joint_state_ except it only contains the joints jog_arm acts on.
  sensor_msgs::JointState internal_joint_state_, original_joint_state_;
  std::map<std::string, std::size_t> joint_state_name_map_;

  std::vector<LowPassFilter> position_filters_;

  // ROS
  ros::Timer timer_;
  ros::Duration period_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber twist_stamped_sub_;
  ros::Subscriber joint_jog_sub_;
  ros::Subscriber collision_velocity_scale_sub_;
  ros::Publisher status_pub_;
  ros::Publisher worst_case_stop_time_pub_;
  ros::Publisher outgoing_cmd_pub_;
  ros::ServiceServer drift_dimensions_server_;
  ros::ServiceServer control_dimensions_server_;

  // Status
  StatusCode status_ = StatusCode::NO_WARNING;
  bool stop_requested_ = false;
  bool paused_ = false;
  bool command_is_stale_ = false;
  bool ok_to_publish_ = false;
  double collision_velocity_scale_ = 1.0;

  // Use ArrayXd type to enable more coefficient-wise operations
  Eigen::ArrayXd delta_theta_;
  Eigen::ArrayXd prev_joint_velocity_;

  const int gazebo_redundant_message_count_ = 30;

  uint num_joints_;

  // True -> allow drift in this dimension. In the command frame. [x, y, z, roll, pitch, yaw]
  std::array<bool, 6> drift_dimensions_ = { { false, false, false, false, false, false } };

  // The dimesions to control. In the command frame. [x, y, z, roll, pitch, yaw]
  std::array<bool, 6> control_dimensions_ = { { true, true, true, true, true, true } };

  // Amount we sleep when waiting
  ros::Rate default_sleep_rate_ = 100;

  // latest_state_mutex_ is used to protect the state below it
  mutable std::mutex latest_state_mutex_;
  Eigen::Isometry3d tf_moveit_to_cmd_frame_;
  geometry_msgs::TwistStampedConstPtr latest_twist_stamped_;
  control_msgs::JointJogConstPtr latest_joint_jog_;
  ros::Time latest_command_stamp_ = ros::Time(0.);
  bool latest_nonzero_twist_stamped_ = false;
  bool latest_nonzero_joint_jog_ = false;
};
}  // namespace moveit_jog_arm
