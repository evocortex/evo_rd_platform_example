//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file BaseControllerROS.h
 * @author evocortex (info@evocortex.com)
 *
 * @brief Interface class to bring the CAN motor stuff into ROS
 *
 * @version 0.1
 * @date 2019-08-14
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#ifndef BASECONTROLLERROS_H
#define BASECONTROLLERROS_H

#include "evo_logger/log/Logger.h"

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

   bool _mecanum_inverted;
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

   // ROS
   ros::NodeHandle _nh;
   ros::Subscriber _sub_cmd_vel;
   ros::Publisher _pub_odom;
   ros::Publisher _pub_enable_signal_off;
   ros::Subscriber _sub_cmd_lift;
   std::vector<ros::Publisher> _pub_lift_pos_vec;

   ros::Rate _loop_rate_hz;

   // TF
   bool _enable_odom_tf;
   tf::TransformBroadcaster _tf_pub_odom;
   std::string _odom_frame_id;
   std::string _odom_child_frame_id;

   bool _error_present;

   bool _is_initialized;

 public:
   BaseControllerROS();

   void init();
   std::vector<MotorShieldConfig> loadConfigROS(ros::NodeHandle& privateNh);
   void main_loop();

   const bool checkStatus(void);

   void publishOdom();
   void cbCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel);
   void checkAndApplyCmdVel();

   void publishLiftPos();
   void cbCmdLift(const std_msgs::Int8::ConstPtr& cmd_lift);
   void checkAndApplyCmdLift();
};

} // namespace evo

#endif // BASECONTROLLERROS_H
