//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

// REV File header format should be defined for company


/**
 * @file BaseControllerROS.h
<<<<<<< HEAD
 * //  REV
 * //  WHA STYLE: wrong author
 * // review: author intern 3 characters (First from name, second and third from surname)
 * @author evocortex (info@evocortex.com) - MMA
 *
 * //  REV
 * //  WHA STYLE: Proposal: "Interface class for accessing CAN motor controls using ROS"
 * @brief Interface class to bring the CAN motor stuff into ROS
=======
 * @author evocortex (info@evocortex.com) - MMA, MBA
 *
 * @brief Base Controller Interface for ROS and EvoRobot com
>>>>>>> devel
 *
 * @version 0.2
 * @date 2020-06-03
 *
<<<<<<< HEAD
 * //  REV
 * //  WHA STYLE: Year of creation or range of years during which this file was edited?
 * @copyright Copyright (c) 2019 Evocortex GmbH
=======
 * @copyright Copyright (c) 2020 Evocortex GmbH
>>>>>>> devel
 *
 */

/*  REV
 *  WHA STYLE: Include the namespace into the guard define name, e.g. "EVO_BASECONTROLLERROS_H"?
 *  MPP STYLE: Suggestion
 *             Use more underscores for the include guard to improve readability.
 * 
 *               EVO_BASE_CONTROLLER_ROS_H
 * 
 * HEN INFO: Or use #pragma once (sadly is not defined as c++ standard, though most compiler support it, reported to be faster)
 */
#ifndef BASECONTROLLERROS_H
#define BASECONTROLLERROS_H

#include "evo_logger/log/Logger.h"

#include "evo_robot_base_interface/LiftController.h"
#include "evo_robot_base_interface/MecanumDrive.h"
#include "evo_robot_base_interface/MotorManager.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
/* MPP INFO: Suggestion
 *           Migration to tf2_ros
 *           See: http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28C%2B%2B%29  
 */
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"

namespace evo {

class BaseControllerROS
{
  /* REV
   * MPP STYLE: * Suggestion
   *              Public members first because they are the interesting part for any
   *              client of this header.
   *            * Missing documentation for public members.
   *            * This class has an awful lot of member variables.
   *              Suggestions: Remove superfluous ones. Try to cluster others.
   * MPP INFO: Dependence on transitive includes potentially problematic?
   *           Examples: Usage of std::string without #include <string> 
   */
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
   bool           _lift_moving;
   bool           _lift_moving_strd;

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
  /* REV
   * MPP STYLE: * Next two functions have a different code style.
   *            * const qualifier of return value and void parameter pointless. 
   *              Copy & paste from C code?  
   */
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

   // lift
   void publishLiftPos();
   void cbCmdLift(const std_msgs::Int8::ConstPtr& cmd_lift);
   void checkAndApplyCmdLift();
};

} // namespace evo

#endif // BASECONTROLLERROS_H
