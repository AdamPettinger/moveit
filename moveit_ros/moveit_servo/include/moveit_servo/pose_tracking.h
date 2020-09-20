/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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
/*
   Author: Andy Zelenak
   Desc: Servoing. Track a pose setpoint in real time.
*/

#pragma once

#include <atomic>
#include <control_toolbox/pid.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.
// All control loops (i.e. moveToPose()) are based on updating the latest
// end_effector_transform_ and receiving the desired target_pose_ which are the
// only two transforms really needed for the PID controllers.
// Each loop cycle should then in general call the following three functions:
//
//   updateControllerSetpoints();
//   updateControllerStateMeasurements();
//   calculateTwistServoCommand();
//
// These will first update the pid controller set points and measurements and then generate
// the twist velocity commands from the controller output which in turn can be passed to Servo.

namespace moveit_servo
{
struct PIDConfig
{
  // Default values
  double dt = 0.001;
  double k_p = 1;
  double k_i = 0;
  double k_d = 0;
  double windup_limit = 0.1;
};

enum PoseTrackingStatusCode : int8_t
{
  SUCCESS = 0,
  NO_RECENT_TARGET_POSE = 1,
  NO_RECENT_END_EFFECTOR_POSE = 2,
  STOP_REQUESTED = 3,
  MOVE_IN_PROGRESS = 4
};

const std::unordered_map<int8_t, std::string>
    POSE_TRACKING_STATUS_CODE_MAP({ { SUCCESS, "Success" },
                                    { NO_RECENT_TARGET_POSE, "No recent target pose" },
                                    { NO_RECENT_END_EFFECTOR_POSE, "No recent end effector pose" },
                                    { STOP_REQUESTED, "Stop requested" },
                                    { MOVE_IN_PROGRESS, "Move in progress" } });

/**
 * Class PoseTracking - subscribe to a target pose.
 * Servo toward the target pose.
 */
class PoseTracking
{
public:
  /** \brief Constructor. Loads ROS parameters under the given namespace. */
  PoseTracking(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
               const std::string& parameter_ns = "");

  /**
   * Starts Servoing towards the goal pose, and blocks until the motion is completed
   *
   * @param positional_tolerance The translational tolerances for when the motion is complete
   * @param angular_tolerance if the rotational tolerance for when the motion is complete
   * @param angular_tolerance if the rotational tolerance for when the motion is complete
   * @return Pose status tracking code of the move
   */
  int8_t moveToPose(const Eigen::Vector3d& positional_tolerance, const double angular_tolerance,
                    const geometry_msgs::PoseStampedConstPtr& target_pose = nullptr);

  /**
   * Asynchronously starts Servoing towards the goal pose
   *
   * @param positional_tolerance The translational tolerances for when the motion is complete
   * @param angular_tolerance if the rotational tolerance for when the motion is complete
   * @param angular_tolerance if the rotational tolerance for when the motion is complete
   */
  void moveToPoseAsync(const Eigen::Vector3d& positional_tolerance, const double angular_tolerance,
                       const geometry_msgs::PoseStampedConstPtr& target_pose = nullptr);

  /** \brief A method for a different thread to stop motion and return early from control loop */
  void stopMotion()
  {
    status_ = STOP_REQUESTED;
    stopTimer();
  }

  /** \brief Change PID parameters. Motion is stopped before the udpate */
  void updatePIDConfig(const double x_proportional_gain, const double x_integral_gain, const double x_derivative_gain,
                       const double y_proportional_gain, const double y_integral_gain, const double y_derivative_gain,
                       const double z_proportional_gain, const double z_integral_gain, const double z_derivative_gain,
                       const double angular_proportional_gain, const double angular_integral_gain,
                       const double angular_derivative_gain);

  void getPIDErrors(double& x_error, double& y_error, double& z_error, double& orientation_error);

  /**
   * Get the End Effector link transform.
   * The transform from the MoveIt planning frame to EE link
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getEEFrameTransform(geometry_msgs::TransformStamped& transform);


  // moveit_servo::Servo instance. Public so we can access member functions like setPaused()
  std::unique_ptr<moveit_servo::Servo> servo_;
  /**
   * Blocks the return until the move to pose task is complete
   *
   * @return The status code of the completed motion
   */
  int8_t blockUntilComplete();

  /**
   * Check if a move is in progress
   *
   * @return True if there is an active target pose and we are moving to it, false otherwise
   */
  bool moveIsActive()
  {
    return timer_is_running_;
  };

private:
  /** \brief Load ROS parameters for controller settings. */
  void readROSParams();

  /** \brief Initialize a PID controller and add it to vector of controllers */
  void initializePID(const PIDConfig& pid_config, std::vector<control_toolbox::Pid>& pid_vector);

  /**
   * Check if XYZ, roll/pitch/yaw tolerances are satisfied
   *
   * @param target_pose The target pose check for
   * @param current_ee_tf The current position of the end effector
   * @param positional_tolerance Tolerances for linear distances
   * @param angular_tolerance Tolerance for angular distance
   * @return True if the pose is sufficiently close
   */
  bool satisfiesPoseTolerance(const geometry_msgs::PoseStamped& target_pose, const Eigen::Isometry3d& current_ee_tf,
                              const Eigen::Vector3d& positional_tolerance, const double angular_tolerance);

  /** \brief Subscribe to the target pose on this topic */
  void targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  /** \brief Main calculation thread */
  void timerCallback(const ros::TimerEvent& timer_event);

  /** \brief Handles the internal tracking for starting the timer */
  void startTimer();

  /** \brief Handles the internal tracking for stopping the timer */
  void stopTimer();  // TODO(adamp): change name away from "timer" to more specificity?

  /** \brief Converts the target pose to the correct frame and stores it */
  bool processIncomingTargetPose(const geometry_msgs::PoseStampedConstPtr& msg);

  /**
   * Use PID controllers to calculate a full spatial velocity toward a pose
   *
   * @param target_pose The target pose to calculate for
   * @param current_ee_tf The current position of the end effector
   * @return A TwistStamped message pointer ready for publishing
   */
  geometry_msgs::TwistStampedConstPtr calculateTwistCommand(const geometry_msgs::PoseStamped& target_pose,
                                                            const Eigen::Isometry3d& current_ee_tf);

  /** \brief Reset flags and PID controllers after a motion completes */
  void doPostMotionReset();

  ros::NodeHandle nh_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model::RobotModelConstPtr robot_model_;
  // Joint group used for controlling the motions
  std::string move_group_name_;

  ros::Timer timer_;
  ros::Duration timer_period_;
  PoseTrackingStatusCode status_;

  // ROS interface to Servo
  ros::Publisher twist_stamped_pub_;

  std::vector<control_toolbox::Pid> cartesian_position_pids_;
  std::vector<control_toolbox::Pid> cartesian_orientation_pids_;
  // Cartesian PID configs
  PIDConfig x_pid_config_, y_pid_config_, z_pid_config_, angular_pid_config_;

  // Transforms w.r.t. planning_frame_
  Eigen::Isometry3d end_effector_transform_;
  geometry_msgs::PoseStamped target_pose_;
  std::unique_ptr<geometry_msgs::PoseStamped> target_pose_ptr_;

  Eigen::Vector3d positional_tolerance_;
  double angular_tolerance_;

  // Subscribe to target pose
  ros::Subscriber target_pose_sub_;

  tf2_ros::Buffer transform_buffer_;
  tf2_ros::TransformListener transform_listener_;

  // Expected frame name, for error checking and transforms
  std::string planning_frame_;

  // Flag that a different thread has requested a stop.
  std::atomic<bool> timer_is_running_;

  // Read parameters from this namespace
  std::string parameter_ns_;

  double angular_error_;
};

// using alias
using PoseTrackingPtr = std::shared_ptr<PoseTracking>;
}  // namespace moveit_servo
