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

#include "moveit_servo/pose_tracking.h"

namespace
{
constexpr char LOGNAME[] = "pose_tracking";
constexpr double DEFAULT_LOOP_PERIOD = 0.01;    // sec
constexpr double ROS_STARTUP_WAIT = 10;         // sec
constexpr double DEFAULT_POSE_TIMEOUT = 0.1;    // sec
constexpr size_t ROS_LOG_THROTTLE_PERIOD = 30;  // Seconds to throttle logs inside loops
}  // namespace

namespace moveit_servo
{
PoseTracking::PoseTracking(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                           const std::string& parameter_ns)
  : planning_scene_monitor_(planning_scene_monitor)
  , timer_period_(DEFAULT_LOOP_PERIOD)
  , status_(NO_RECENT_TARGET_POSE)
  , angular_tolerance_(0)
  , transform_listener_(transform_buffer_)
  , timer_is_running_(false)
  , parameter_ns_(parameter_ns)
  , angular_error_(0)
{
  readROSParams();

  positional_tolerance_.setZero();

  robot_model_ = planning_scene_monitor_->getRobotModel();

  // Initialize PID controllers
  initializePID(x_pid_config_, cartesian_position_pids_);
  initializePID(y_pid_config_, cartesian_position_pids_);
  initializePID(z_pid_config_, cartesian_position_pids_);
  initializePID(angular_pid_config_, cartesian_orientation_pids_);

  // Use the C++ interface that Servo provides
  servo_ = std::make_unique<moveit_servo::Servo>(nh_, planning_scene_monitor_, parameter_ns_);
  servo_->start();

  // Connect to Servo ROS interfaces
  std::string target_pose_topic = "/" + parameter_ns_ + "/target_pose";
  target_pose_sub_ =
      nh_.subscribe<geometry_msgs::PoseStamped>(target_pose_topic, 1, &PoseTracking::targetPoseCallback, this);

  // Publish outgoing twist commands to the Servo object
  twist_stamped_pub_ =
      nh_.advertise<geometry_msgs::TwistStamped>(servo_->getParameters().cartesian_command_in_topic, 1);
}

void PoseTracking::timerCallback(const ros::TimerEvent& timer_event)
{
  // Check for an active target pose
  if (target_pose_ptr_)
  {
    // Copy the target pose to use for this cycle
    target_pose_ = *target_pose_ptr_;
  }
  else
  {
    status_ = NO_RECENT_TARGET_POSE;
    // TODO(adamp): track how long we are in this state and warn user if too long
    return;
  }

  // Make sure we have a valid pose from the Servo
  if (!servo_->getEEFrameTransform(end_effector_transform_))
  {
    status_ = NO_RECENT_END_EFFECTOR_POSE;
    ROS_WARN_THROTTLE_NAMED(ROS_LOG_THROTTLE_PERIOD, LOGNAME, "Could not get end effector pose from Servo!");
    // TODO(adamp): decide if we stop the timer or just skip this CB cycle
    return;
  }

  // Check to see if we have reached the target pose
  if (satisfiesPoseTolerance(target_pose_, end_effector_transform_, positional_tolerance_, angular_tolerance_))
  {
    status_ = SUCCESS;
    stopTimer();
    return;
  }

  // If we made it here, we need to do the actual math and move the robot
  status_ = MOVE_IN_PROGRESS;

  // Compute servo command from PID controller output
  auto msg = calculateTwistCommand(target_pose_, end_effector_transform_);

  // Send command to the Servo object, for execution
  twist_stamped_pub_.publish(*msg);
}

void PoseTracking::moveToPoseAsync(const Eigen::Vector3d& positional_tolerance, const double angular_tolerance,
                                   const geometry_msgs::PoseStampedConstPtr& target_pose)
{
  // Set the passed pose as the target pose if it's not null
  if (target_pose)
  {
    processIncomingTargetPose(target_pose);
  }

  // Set the tolerances for the move
  positional_tolerance_ = positional_tolerance;
  angular_tolerance_ = angular_tolerance;

  // Start the timer if it's not already running
  if (!timer_is_running_)
  {
    startTimer();
  }
}

int8_t PoseTracking::moveToPose(const Eigen::Vector3d& positional_tolerance, const double angular_tolerance,
                                const geometry_msgs::PoseStampedConstPtr& target_pose)
{
  moveToPoseAsync(positional_tolerance, angular_tolerance, target_pose);
  return blockUntilComplete();
}

int8_t PoseTracking::blockUntilComplete()
{
  // Wait until ROS dies or the timer is not running
  ros::Duration wait_period(0.001);
  while (ros::ok() && timer_is_running_)
  {
    wait_period.sleep();
  }

  // Then return the current status
  return status_;
}

void PoseTracking::startTimer()
{
  timer_is_running_ = true;
  timer_ = nh_.createTimer(timer_period_, &PoseTracking::timerCallback, this);
}

void PoseTracking::stopTimer()
{
  timer_is_running_ = false;
  timer_.stop();
  target_pose_ptr_.reset();
  doPostMotionReset();
}

void PoseTracking::readROSParams()
{
  std::size_t error = 0;

  // Check for parameter namespace from launch file. All other parameters will be read from this namespace.
  std::string yaml_namespace;
  if (ros::param::get("~parameter_ns", yaml_namespace))
  {
    if (!parameter_ns_.empty())
      ROS_WARN_STREAM_NAMED(LOGNAME,
                            "A parameter namespace was specified in the launch file AND in the constructor argument.");

    parameter_ns_ = yaml_namespace;
  }

  // Wait for ROS parameters to load
  ros::Time begin = ros::Time::now();
  while (ros::ok() && !ros::param::has(parameter_ns_ + "/planning_frame") &&
         ((ros::Time::now() - begin).toSec() < ROS_STARTUP_WAIT))
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "Waiting for parameter: " << parameter_ns_ + "/planning_frame");
    ros::Duration(0.1).sleep();
  }

  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/planning_frame", planning_frame_);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/move_group_name", move_group_name_);
  if (!planning_scene_monitor_->getRobotModel()->hasJointModelGroup(move_group_name_))
  {
    ++error;
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Unable to find the specified joint model group: " << move_group_name_);
  }

  double publish_period;
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/publish_period", publish_period);
  timer_period_ = ros::Duration(publish_period);

  x_pid_config_.dt = publish_period;
  y_pid_config_.dt = publish_period;
  z_pid_config_.dt = publish_period;
  angular_pid_config_.dt = publish_period;

  double windup_limit;
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/windup_limit", windup_limit);
  x_pid_config_.windup_limit = windup_limit;
  y_pid_config_.windup_limit = windup_limit;
  z_pid_config_.windup_limit = windup_limit;
  angular_pid_config_.windup_limit = windup_limit;

  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/x_proportional_gain", x_pid_config_.k_p);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/y_proportional_gain", y_pid_config_.k_p);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/z_proportional_gain", z_pid_config_.k_p);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/x_integral_gain", x_pid_config_.k_i);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/y_integral_gain", y_pid_config_.k_i);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/z_integral_gain", z_pid_config_.k_i);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/x_derivative_gain", x_pid_config_.k_d);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/y_derivative_gain", y_pid_config_.k_d);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/z_derivative_gain", z_pid_config_.k_d);

  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/angular_proportional_gain", angular_pid_config_.k_p);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/angular_integral_gain", angular_pid_config_.k_i);
  error += !rosparam_shortcuts::get("", nh_, parameter_ns_ + "/angular_derivative_gain", angular_pid_config_.k_d);

  rosparam_shortcuts::shutdownIfError(ros::this_node::getName(), error);
}

void PoseTracking::initializePID(const PIDConfig& pid_config, std::vector<control_toolbox::Pid>& pid_vector)
{
  bool use_anti_windup = true;
  pid_vector.push_back(control_toolbox::Pid(pid_config.k_p, pid_config.k_i, pid_config.k_d, -pid_config.windup_limit,
                                            pid_config.windup_limit, use_anti_windup));
}

bool PoseTracking::satisfiesPoseTolerance(const geometry_msgs::PoseStamped& target_pose,
                                          const Eigen::Isometry3d& current_ee_tf,
                                          const Eigen::Vector3d& positional_tolerance, const double angular_tolerance)
{
  double x_error = target_pose.pose.position.x - current_ee_tf.translation()(0);
  double y_error = target_pose.pose.position.y - current_ee_tf.translation()(1);
  double z_error = target_pose.pose.position.z - current_ee_tf.translation()(2);

  return (fabs(x_error) < positional_tolerance(0)) && (fabs(y_error) < positional_tolerance(1)) &&
         (fabs(z_error) < positional_tolerance(2) && fabs(angular_error_) < angular_tolerance);
}

void PoseTracking::targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // We only want to set the incoming pose as active if the timer is actually running
  if (timer_is_running_)
  {
    processIncomingTargetPose(msg);
  }
}

bool PoseTracking::processIncomingTargetPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  target_pose_ptr_ = std::make_unique<geometry_msgs::PoseStamped>(*msg);

  // Transform to MoveIt planning frame
  if (target_pose_ptr_->header.frame_id != planning_frame_)
  {
    auto target_to_planning_frame = transform_buffer_.lookupTransform(
        planning_frame_, target_pose_ptr_->header.frame_id, ros::Time(0), ros::Duration(0.1));
    tf2::doTransform(*target_pose_ptr_, *target_pose_ptr_, target_to_planning_frame);
  }
  target_pose_ptr_->header.stamp = ros::Time::now();

  return true;
}

geometry_msgs::TwistStampedConstPtr PoseTracking::calculateTwistCommand(const geometry_msgs::PoseStamped& target_pose,
                                                                        const Eigen::Isometry3d& current_ee_tf)
{
  // use the shared pool to create a message more efficiently
  auto msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
  msg->header.frame_id = target_pose.header.frame_id;

  // Get twist components from PID controllers
  geometry_msgs::Twist& twist = msg->twist;

  // Position
  twist.linear.x = cartesian_position_pids_[0].computeCommand(
      target_pose.pose.position.x - current_ee_tf.translation()(0), timer_period_);
  twist.linear.y = cartesian_position_pids_[1].computeCommand(
      target_pose.pose.position.y - current_ee_tf.translation()(1), timer_period_);
  twist.linear.z = cartesian_position_pids_[2].computeCommand(
      target_pose.pose.position.z - current_ee_tf.translation()(2), timer_period_);

  // Orientation algorithm:
  // - Find the orientation error as a quaternion: q_error = q_desired * q_current ^ -1
  // - Use the quaternion PID controllers to calculate a quaternion rate, q_error_dot
  // - Convert to angular velocity for the TwistStamped message
  Eigen::Quaterniond q_desired(target_pose.pose.orientation.w, target_pose.pose.orientation.x,
                               target_pose.pose.orientation.y, target_pose.pose.orientation.z);
  Eigen::Quaterniond q_current(current_ee_tf.rotation());
  Eigen::Quaterniond q_error = q_desired * q_current.inverse();

  // Convert axis-angle to angular velocity
  Eigen::AngleAxisd axis_angle(q_error);
  // Cache the angular error, for rotation tolerance checking
  angular_error_ = axis_angle.angle();
  double ang_vel_magnitude = cartesian_orientation_pids_[0].computeCommand(angular_error_, timer_period_);
  twist.angular.x = ang_vel_magnitude * axis_angle.axis()[0];
  twist.angular.y = ang_vel_magnitude * axis_angle.axis()[1];
  twist.angular.z = ang_vel_magnitude * axis_angle.axis()[2];

  msg->header.stamp = ros::Time::now();

  return msg;
}

void PoseTracking::doPostMotionReset()
{
  angular_error_ = 0;

  // Reset error integrals and previous errors of PID controllers
  cartesian_position_pids_[0].reset();
  cartesian_position_pids_[1].reset();
  cartesian_position_pids_[2].reset();
  cartesian_orientation_pids_[0].reset();
}

void PoseTracking::updatePIDConfig(const double x_proportional_gain, const double x_integral_gain,
                                   const double x_derivative_gain, const double y_proportional_gain,
                                   const double y_integral_gain, const double y_derivative_gain,
                                   const double z_proportional_gain, const double z_integral_gain,
                                   const double z_derivative_gain, const double angular_proportional_gain,
                                   const double angular_integral_gain, const double angular_derivative_gain)
{
  stopMotion();

  x_pid_config_.k_p = x_proportional_gain;
  x_pid_config_.k_i = x_integral_gain;
  x_pid_config_.k_d = x_derivative_gain;
  y_pid_config_.k_p = y_proportional_gain;
  y_pid_config_.k_i = y_integral_gain;
  y_pid_config_.k_d = y_derivative_gain;
  z_pid_config_.k_p = z_proportional_gain;
  z_pid_config_.k_i = z_integral_gain;
  z_pid_config_.k_d = z_derivative_gain;

  cartesian_position_pids_.clear();
  cartesian_orientation_pids_.clear();
  initializePID(x_pid_config_, cartesian_position_pids_);
  initializePID(y_pid_config_, cartesian_position_pids_);
  initializePID(z_pid_config_, cartesian_position_pids_);
  initializePID(angular_pid_config_, cartesian_orientation_pids_);

  doPostMotionReset();
}

void PoseTracking::getPIDErrors(double& x_error, double& y_error, double& z_error, double& orientation_error)
{
  double dummy1, dummy2;
  cartesian_position_pids_.at(0).getCurrentPIDErrors(&x_error, &dummy1, &dummy2);
  cartesian_position_pids_.at(1).getCurrentPIDErrors(&y_error, &dummy1, &dummy2);
  cartesian_position_pids_.at(2).getCurrentPIDErrors(&z_error, &dummy1, &dummy2);
  cartesian_orientation_pids_.at(0).getCurrentPIDErrors(&orientation_error, &dummy1, &dummy2);
}

bool PoseTracking::getEEFrameTransform(geometry_msgs::TransformStamped& transform)
{
  return servo_->getEEFrameTransform(transform);
}
}  // namespace moveit_servo
