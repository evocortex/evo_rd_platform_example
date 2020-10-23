//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file BaseControllerROS.h
 * @author evocortex (info@evocortex.com) - MMA, MBA
 *
 * @brief Base Controller Interface for ROS and EvoRobot com
 *
 * @version 0.2
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2020 Evocortex GmbH
 *
 */

#ifndef BASECONTROLLERROS_H
#define BASECONTROLLERROS_H

#include "evo_logger/log/Logger.h"

#include "evo_mbed/Version.h"

#include "evo_robot_base_interface/LiftController.h"
#include "evo_robot_base_interface/MecanumDrive.h"
#include "evo_robot_base_interface/MotorManager.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

namespace evo {

class BaseControllerROS
{
 private:
   std::string _logger_prefix;

   // Motor Manager
   MotorManager _motor_handler;

   // Mecanum Drive
   MecanumDrive _mecanum_drive;
   MecanumCovariance _mecanum_covariance;

   double _timeout_cmd_vel;
   ros::Time _stamp_cmd_vel;
   MecanumVel _cmd_vel;
   MecanumPose _odom_pose;

   // Lift Controller
   LiftController _lift_controller;
   bool           _lift_active;

   double _timeout_cmd_lift;
   ros::Time _stamp_cmd_lift;
   int8_t _cmd_lift;
   bool _enable_lift_control;

   // ROS
   ros::NodeHandle _nh;
   ros::Subscriber _sub_cmd_vel;
   ros::Publisher _pub_odom;
   ros::Publisher _pub_enable_signal_off;
   ros::Subscriber _sub_cmd_lift;
   std::vector<ros::Publisher> _pub_lift_pos_vec;

   ros::Rate _loop_rate_hz;

   // Joint state publisher 
   bool _enable_joint_state_publisher;
   ros::Publisher _pub_joint_state;
   sensor_msgs::JointState _joint_state_msg;

   // TF
   bool _enable_odom_tf;
   tf::TransformBroadcaster _tf_pub_odom;
   std::string _odom_frame_id;
   std::string _odom_child_frame_id;

   bool _error_present;

   bool _is_initialized;

 public:
   BaseControllerROS();

   bool init();
   std::vector<MotorShieldConfig> loadConfigROS(ros::NodeHandle& privateNh);
   void main_loop();

   const bool checkStatus(void);

   void publishBaseStatus();

   // drives
   void publishOdom();
   void publishOdomMsg(const MecanumVel& odom_vel, const MecanumPose& odom_pose);
   void publishOdomTF(const MecanumPose& odom_pose);
   void publishJointStates(const MecanumWheelData& wheel_positions, const MecanumWheelData& wheel_rotations);


   void cbCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel);
   void checkAndApplyCmdVel();

   bool checkFirmwareVersion(const int major_ver, const int minor_ver, const int patch_ver);
   // lift
   void publishLiftPos();
   void cbCmdLift(const std_msgs::Int8::ConstPtr& cmd_lift);
   void checkAndApplyCmdLift();
};

} // namespace evo

#endif // BASECONTROLLERROS_H
