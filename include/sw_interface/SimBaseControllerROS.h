//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file SimBaseControllerROS.h
 * @author evocortex (info@evocortex.com)
 *
 * @brief Class for simulation of the R&D platform
 *
 * @version 0.1
 * @date 2020-05-12
 *
 * @copyright Copyright (c) 2020 Evocortex GmbH
 *
 */

#pragma once

#include "evo_logger/log/Logger.h"
#include "evo_robot_base_interface/MecanumDrive.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

namespace evo {

class SimBaseControllerROS
{
 private:
   std::string _logger_prefix;

   // Mecanum Drive
   MecanumDrive _mecanum_drive;
   MecanumCovariance _mecanum_covariance;

   double _timeout_cmd_vel;
   ros::Time _stamp_cmd_vel;
   MecanumVel _cmd_vel;
   MecanumPose _odom_pose;

   // ROS
   ros::NodeHandle _nh;
   ros::Subscriber _sub_cmd_vel;
   ros::Publisher _pub_odom;

    MecanumWheelData _last_wheel_positions;
   ros::Subscriber _sub_sim_wheeldata;
   std::map<MOTOR_MAPPING_MECANUM, ros::Publisher> _map_pub_sim_wheeldata;

   std::map<MOTOR_MAPPING_MECANUM, std::string> _map_name_to_pos;

   ros::Rate _loop_rate_hz;

   // TF
   bool _enable_odom_tf;
   tf::TransformBroadcaster _tf_pub_odom;
   std::string _odom_frame_id;
   std::string _odom_child_frame_id;

 public:
   SimBaseControllerROS();
   ~SimBaseControllerROS();

   void main_loop();
   void cbJointState(const sensor_msgs::JointState::ConstPtr& jointstate);
   void publishOdomMsg(const MecanumVel& odom_vel, const MecanumPose& odom_pose);
   void publishOdomTF(const MecanumPose& odom_pose);


   void cbCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel);
   void checkAndApplyCmdVel();
};

} // namespace evo