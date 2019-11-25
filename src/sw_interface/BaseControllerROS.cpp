//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file BaseControllerROS.cpp
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

#include "sw_interface/BaseControllerROS.h"

namespace evo {

BaseControllerROS::BaseControllerROS() :
    _logger_prefix("BaseControllerROS: "), _error_present(false),
    _is_initialized(false), _loop_rate_hz(50)
{
   evo::log::init("");
}

void BaseControllerROS::init()
{
   evo::log::get() << _logger_prefix << "start init process!" << evo::info;

   bool success = true;
   // load all relevant parameters
   ros::NodeHandle privateNh("~");

   // parameters for the motor manager
   std::string can_interface_name;
   privateNh.param("can_interface_name", can_interface_name,
                   std::string("can-motor"));
   if(!_motor_handler.initCanInterface(can_interface_name))
   {
      evo::log::get() << _logger_prefix
                      << "initialization of the CAN interface failed!" << evo::error;
      evo::log::get() << _logger_prefix << "Check if the interfacename ["
                      << can_interface_name << "] is correct!" << evo::error;
      evo::log::get() << _logger_prefix << "--> Shutdown" << evo::error;
      exit(1);
   }
   _motor_handler.setConfig(loadConfigROS(privateNh));
   success &= _motor_handler.initFromConfig();
   _motor_handler.initMotorMapping(_mecanum_drive, _lift_controller);
   success &= _motor_handler.enableAllMotors();

   // parameters for the mecanum drive
   double wheel_radius_in_m, wheel_distance_front_back_in_m,
       wheel_distance_left_right_in_m;
   privateNh.param("wheel_radius_in_m", wheel_radius_in_m, 0.0);
   privateNh.param("wheel_distance_front_back_in_m", wheel_distance_front_back_in_m,
                   0.0);
   privateNh.param("wheel_distance_left_right_in_m", wheel_distance_left_right_in_m,
                   0.0);
   _mecanum_drive.setWheelRadiusInM(wheel_radius_in_m);
   _mecanum_drive.setWheelDistanceFrontBackInM(wheel_distance_front_back_in_m);
   _mecanum_drive.setWheelDistanceLeftRightInM(wheel_distance_left_right_in_m);
   success &= _mecanum_drive.checkInitState();

   if(!success)
   {
      evo::log::get() << _logger_prefix << "init process not successful!"
                      << evo::error;
      return;
   }

   // covariances
   privateNh.param("covariance_pos_x", _mecanum_covariance.cov_pos_x, 1.0);
   privateNh.param("covariance_pos_y", _mecanum_covariance.cov_pos_y, 1.0);
   privateNh.param("covariance_pos_yaw", _mecanum_covariance.cov_pos_yaw, 1.0);
   privateNh.param("covariance_vel_x", _mecanum_covariance.cov_vel_x, 1.0);
   privateNh.param("covariance_vel_y", _mecanum_covariance.cov_vel_y, 1.0);
   privateNh.param("covariance_vel_yaw", _mecanum_covariance.cov_vel_yaw, 1.0);

   // debug drives
   bool debug_motor_mapping = false;
   privateNh.param("debug_motor_mapping", debug_motor_mapping, false);
   if(debug_motor_mapping)
      _mecanum_drive.debugMotorMapping();
   privateNh.param("mecanum_inverted", _mecanum_inverted, false);

   // parameters for this class
   std::string topic_sub_cmd_vel, topic_sub_cmd_lift, topic_pub_odom,
       topic_pub_enable_signal_off;
   double com_timeout_s, loop_rate_hz;
   privateNh.param("loop_rate_hz", loop_rate_hz, 50.0);
   _loop_rate_hz = ros::Rate(loop_rate_hz);

   privateNh.param("topic_pub_odom", topic_pub_odom, std::string("odom"));
   privateNh.param("topic_pub_enable_signal_off", topic_pub_enable_signal_off,
                   std::string("enable_signal_off"));
   privateNh.param("topic_sub_cmd_vel", topic_sub_cmd_vel, std::string("cmd_vel"));
   privateNh.param("topic_sub_cmd_lift", topic_sub_cmd_lift,
                   std::string("cmd_lift"));

   privateNh.param("com_timeout_s", com_timeout_s, 0.1);
   privateNh.param("cmd_vel_timeout_s", _timeout_cmd_vel, 0.1);
   privateNh.param("cmd_lift_timeout_s", _timeout_cmd_lift, 0.5);

   privateNh.param("enable_odom_tf", _enable_odom_tf, true);
   privateNh.param("odom_frame_id", _odom_frame_id, std::string("odom"));
   privateNh.param("odom_child_frame_id", _odom_child_frame_id,
                   std::string("base_footprint"));

   // setup connections
   _sub_cmd_vel = _nh.subscribe<geometry_msgs::Twist>(
       topic_sub_cmd_vel, 1, &BaseControllerROS::cbCmdVel, this);
   _pub_odom = _nh.advertise<nav_msgs::Odometry>(topic_pub_odom, 1);
   _pub_enable_signal_off =
       _nh.advertise<std_msgs::Bool>(topic_pub_enable_signal_off, 1);
   _sub_cmd_lift = _nh.subscribe<std_msgs::Int8>(
       topic_sub_cmd_lift, 1, &BaseControllerROS::cbCmdLift, this);

   // finish flag
   evo::log::get() << _logger_prefix << "finished init process!" << evo::info;
   _is_initialized = true;
}

std::vector<MotorShieldConfig>
BaseControllerROS::loadConfigROS(ros::NodeHandle& privateNh)
{
   std::vector<MotorShieldConfig> mc_config_ros;
   std::string paramName, paramPrefix;
   int controller_id         = 1;
   static const int n_motors = 2;

   std::map<std::string, double> param_map;

   // check if the next motorshield exists
   paramPrefix = "ms" + std::to_string(controller_id);
   while(privateNh.hasParam(paramPrefix + "/enable"))
   {
      bool enable_mc = false;
      privateNh.getParam(paramPrefix + "/enable", enable_mc);
      evo::log::get() << _logger_prefix << "Enable ms" << controller_id << ": "
                      << enable_mc << evo::info;

      if(enable_mc)
      {
         evo::log::get() << _logger_prefix << "--------------------------"
                         << evo::info;
         evo::log::get() << _logger_prefix << "Loading parameters for ms"
                         << controller_id << evo::info;

         // load mc param
         MotorShieldConfig mc_config;
         mc_config.id   = controller_id;
         int timeout_ms = 10;
         if(!privateNh.getParam(paramPrefix + "/timeout_ms", timeout_ms))
         {
            timeout_ms = 10;
            evo::log::get() << _logger_prefix
                            << "no timeout_ms parameter given! using default: "
                            << mc_config.timeout_ms << evo::warn;
         }
         mc_config.timeout_ms = static_cast<uint32_t>(timeout_ms);
         // could also be loaded as param
         mc_config.n_motors = n_motors;
         mc_config.motor_configs.clear();

         // load params for two motors
         for(int motor_id = 0; motor_id < mc_config.n_motors; motor_id++)
         {
            evo::log::get() << _logger_prefix << "Loading parameters for motor"
                            << motor_id << evo::info;
            // search for these params
            param_map["type"]          = 0.0;
            param_map["ctrl_mode"]     = 0.0;
            param_map["kp"]            = 0.0;
            param_map["ki"]            = 0.0;
            param_map["kd"]            = 0.0;
            param_map["pwm_limit"]     = 0.0;
            param_map["gear_ratio"]    = 0.0;
            param_map["encoder_res"]   = 0.0;
            param_map["motor_mapping"] = 0.0;

            for(auto& param : param_map)
            {
               paramName = paramPrefix + "/motor" + std::to_string(motor_id) + "/" +
                           param.first;
               if(privateNh.hasParam(paramName))
               {
                  privateNh.getParam(paramName, param.second);
                  evo::log::get()
                      << _logger_prefix << "received Param: " << param.first << " = "
                      << param.second << evo::info;
               }
               else
               {
                  evo::log::get()
                      << _logger_prefix
                      << "failed to retrieve param with name: " << paramName
                      << evo::warn;
                  evo::log::get() << _logger_prefix << "using val: " << param.second
                                  << evo::warn;
               }
            }
            MotorConfig motor_config;
            motor_config.type =
                static_cast<evo_mbed::MotorType>(param_map.at("type"));
            motor_config.mode =
                static_cast<evo_mbed::MotorControlMode>(param_map.at("ctrl_mode"));
            motor_config.kp          = param_map.at("kp");
            motor_config.ki          = param_map.at("ki");
            motor_config.kd          = param_map.at("kd");
            motor_config.encoder_res = param_map.at("encoder_res");
            motor_config.gear_ratio  = param_map.at("gear_ratio");
            motor_config.pwm_limit   = param_map.at("pwm_limit");
            motor_config.motor_mapping =
                static_cast<uint8_t>(param_map.at("motor_mapping"));
            mc_config.motor_configs.push_back(motor_config);
         }
         mc_config_ros.push_back(mc_config);
      }
      ++controller_id;
      paramPrefix = "ms" + std::to_string(controller_id);
   }
   return mc_config_ros;
}

void BaseControllerROS::publishOdom()
{
   MecanumVel odom_raw = _mecanum_drive.getOdom();

   nav_msgs::Odometry odom;
   odom.header.stamp          = ros::Time::now();
   odom.twist.twist.linear.x  = odom_raw._x_ms;
   odom.twist.twist.linear.y  = odom_raw._y_ms;
   odom.twist.twist.angular.z = odom_raw._yaw_rads;

   // add up to position - from vel
   //_odom_pose.updatePoseFromVel(odom_raw, _loop_rate_hz.cycleTime().toSec());

   // add up position - from increment
   _odom_pose.updatePoseFromIncrement(_mecanum_drive.getPoseIncrement());

   odom.pose.pose.position.x     = _odom_pose._x_m;
   odom.pose.pose.position.y     = _odom_pose._y_m;
   tf::Quaternion pose_quaterion = tf::createQuaternionFromYaw(_odom_pose._yaw_rad);
   odom.pose.pose.orientation.w  = pose_quaterion.getW();
   odom.pose.pose.orientation.y  = pose_quaterion.getY();
   odom.pose.pose.orientation.z  = pose_quaterion.getZ();
   odom.pose.pose.orientation.x  = pose_quaterion.getX();

   // covariances
   const double cpx   = _mecanum_covariance.cov_pos_x;
   const double cpy   = _mecanum_covariance.cov_pos_y;
   const double cpyaw = _mecanum_covariance.cov_pos_yaw;
   const double cvx   = _mecanum_covariance.cov_vel_x;
   const double cvy   = _mecanum_covariance.cov_vel_y;
   const double cvyaw = _mecanum_covariance.cov_vel_yaw;

   odom.twist.covariance = {cpx, 0.0, 0.0, 0.0, 0.0, 0.0,   0.0f, cpy, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, cpyaw, 0.0,  0.0, 0.0,
                            0.0, 0.0, 0.0, cvx, 0.0, 0.0,   0.0,  0.0, 0.0,
                            0.0, cvy, 0.0, 0.0, 0.0, 0.0,   0.0,  0.0, cvyaw};

   odom.pose.covariance = odom.twist.covariance;
   _pub_odom.publish(odom);

   // Transform
   if(_enable_odom_tf)
   {
      tf::StampedTransform tf_odom;
      tf_odom.setOrigin(tf::Vector3(_odom_pose._x_m, _odom_pose._y_m, 0));
      tf_odom.setRotation(pose_quaterion);
      tf_odom.frame_id_       = _odom_frame_id;
      tf_odom.child_frame_id_ = _odom_child_frame_id;
      tf_odom.stamp_          = odom.header.stamp;
      _tf_pub_odom.sendTransform(tf_odom);
   }
}

void BaseControllerROS::cbCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
   _stamp_cmd_vel     = ros::Time::now();
   _cmd_vel._x_ms     = cmd_vel->linear.x;
   _cmd_vel._y_ms     = cmd_vel->linear.y;
   _cmd_vel._yaw_rads = cmd_vel->angular.z;
}

void BaseControllerROS::cbCmdLift(const std_msgs::Int8::ConstPtr& cmd_lift)
{
   _stamp_cmd_lift = ros::Time::now();
   _cmd_lift       = cmd_lift->data;
}

void BaseControllerROS::checkAndApplyCmdVel()
{
   // check timestamp
   if(ros::Time::now().toSec() > (_stamp_cmd_vel.toSec() + _timeout_cmd_vel))
   {
      evo::log::get() << _logger_prefix
                      << "cmd vel timeout detected! stopping robot.." << evo::warn;
      MecanumVel zero;
      _mecanum_drive.setTargetSpeed(zero);
   }
   else
   {
      // check limits for max speed?
      _mecanum_drive.setTargetSpeed(_cmd_vel);
   }
}

void BaseControllerROS::checkAndApplyCmdLift()
{
   // check timestamp
   if(ros::Time::now().toSec() > (_stamp_cmd_lift.toSec() + _timeout_cmd_lift))
   {
      evo::log::get() << _logger_prefix
                      << "cmd lift timeout detected! stopping lift mechanism.."
                      << evo::warn;
      _lift_controller.setMovingDirection(0);
   }
   else
   {
      _lift_controller.setMovingDirection(_cmd_lift);
   }
}

/*
 * -> Removed by bausma
 * -> Function handled by checkStatus() function
void BaseControllerROS::cbSyncTimeout(const ros::TimerEvent &te)
{
    std_msgs::Bool enable_signal_off;
    enable_signal_off.data = false;
    if(!_motor_handler.checkSyncStatus())
    {
        evo::log::get() << _logger_prefix << "motorshield out of sync! --> resync" <<
evo::warn; if(!_motor_handler.resyncMotorShields())
        {
            evo::log::get() << _logger_prefix << "resync failed! -> Missing enable
signal?" << evo::error; enable_signal_off.data = true;
        }
        else
        {
            _motor_handler.enableAllMotors();
        }
    }
    _pub_enable_signal_off.publish(enable_signal_off);
}
*/

const bool BaseControllerROS::checkStatus()
{
   std_msgs::Bool enable_signal_off;
   enable_signal_off.data = false;

   if(!_motor_handler.checkStatus())
   {
      evo::log::get()
          << _logger_prefix
          << " motorshield communication error! -> Missing enable signal?"
          << evo::warn;
      _error_present         = true;
      enable_signal_off.data = true;

      _motor_handler.disableAllMotors();
   }
   // falling edge detection for error status to re-enable motors
   else if(_error_present)
   {
      _error_present = false;

      // Small hack
      std::this_thread::sleep_for(std::chrono::milliseconds(500u));

      _motor_handler.enableAllMotors();
   }

   _pub_enable_signal_off.publish(enable_signal_off);

   return !_error_present;
}

void BaseControllerROS::main_loop()
{
   if(!_is_initialized)
   {
      evo::log::get() << _logger_prefix << "not initialized! check your code!"
                      << evo::error;
      return;
   }
   else
   {
      evo::log::get() << _logger_prefix << "starting main control loop.."
                      << evo::info;
      _motor_handler.enableAllMotors();
      while(ros::ok())
      {
         ros::spinOnce();
         publishOdom();

         if(checkStatus())
         {
            checkAndApplyCmdVel();
            checkAndApplyCmdLift();
         }

         _loop_rate_hz.sleep();
      }
   }
}

} // namespace evo
