//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file BaseControllerROS.cpp
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

#include "sw_interface/BaseControllerROS.h"

namespace evo {

BaseControllerROS::BaseControllerROS() :
    _logger_prefix("BaseControllerROS: "), 
    _lift_active(false),
    _error_present(false), 
    _is_initialized(false),
    _loop_rate_hz(50)
{
   evo::log::init("");
}

bool BaseControllerROS::init()
{
   evo::log::get() << _logger_prefix << "start init process!" << evo::info;

   bool success = true;
   // load all relevant parameters
   ros::NodeHandle privateNh("~");

   // parameters for the motor manager
   std::string can_interface_name;
   privateNh.param("can_interface_name", can_interface_name, std::string("can-motor"));
   if(!_motor_handler.initCanInterface(can_interface_name))
   {
      evo::log::get() << _logger_prefix
                      << "initialization of the CAN interface failed!" << evo::error;
      evo::log::get() << _logger_prefix << "Check if the interfacename ["
                      << can_interface_name << "] is correct!" << evo::error;
      evo::log::get() << _logger_prefix << "--> Exit" << evo::error;
      return false;
   }
   _motor_handler.setConfig(loadConfigROS(privateNh));
   success &= _motor_handler.initFromConfig();
   _motor_handler.initMotorMapping(_mecanum_drive, _lift_controller);
   success &= _motor_handler.enableAllMotors();

   // parameters for the mecanum drive
   double wheel_radius_in_m, wheel_distance_front_back_in_m, wheel_distance_left_right_in_m;
   privateNh.param("wheel_radius_in_m", wheel_radius_in_m, 0.0);
   privateNh.param("wheel_distance_front_back_in_m", wheel_distance_front_back_in_m, 0.0);
   privateNh.param("wheel_distance_left_right_in_m", wheel_distance_left_right_in_m, 0.0);
   _mecanum_drive.setWheelRadiusInM(wheel_radius_in_m);
   _mecanum_drive.setWheelDistanceFrontBackInM(wheel_distance_front_back_in_m);
   _mecanum_drive.setWheelDistanceLeftRightInM(wheel_distance_left_right_in_m);
   success &= _mecanum_drive.checkInitState();

   if(!success)
   {
      evo::log::get() << _logger_prefix << "init process not successful!"
                      << evo::error;
      return false;
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

   // parameters for this class
   std::string topic_sub_cmd_vel, topic_sub_cmd_lift, topic_pub_odom, topic_pub_enable_signal_off;
   double com_timeout_s, loop_rate_hz;
   privateNh.param("loop_rate_hz", loop_rate_hz, 50.0);
   _loop_rate_hz = ros::Rate(loop_rate_hz);

   // regular topics
   privateNh.param("topic_pub_odom", topic_pub_odom, std::string("odom"));
   privateNh.param("topic_pub_enable_signal_off", topic_pub_enable_signal_off, std::string("enable_signal_off"));
   privateNh.param("topic_sub_cmd_vel", topic_sub_cmd_vel, std::string("cmd_vel"));
   privateNh.param("topic_sub_cmd_lift", topic_sub_cmd_lift, std::string("cmd_lift"));

   // timeouts
   privateNh.param("com_timeout_s", com_timeout_s, 0.1);
   privateNh.param("cmd_vel_timeout_s", _timeout_cmd_vel, 0.1);
   privateNh.param("cmd_lift_timeout_s", _timeout_cmd_lift, 0.5);

   // odometry
   privateNh.param("enable_odom_tf", _enable_odom_tf, true);
   privateNh.param("odom_frame_id", _odom_frame_id, std::string("odom"));
   privateNh.param("odom_child_frame_id", _odom_child_frame_id, std::string("base_footprint"));

   privateNh.param("enable_lift_control", _enable_lift_control, false);

   // toggle joint state publishing
   privateNh.param("enable_joint_state_publisher", _enable_joint_state_publisher, false);
   if(_enable_joint_state_publisher)
   {
      int n_joints = 0;
      std::string topic_joint_states;
      privateNh.param("topic_pub_joint_states", topic_joint_states, std::string("base_joint_states"));
      _pub_joint_state = _nh.advertise<sensor_msgs::JointState>(topic_joint_states, 1);
      _joint_state_msg.name.push_back("joint_wheel_front_left");
      _joint_state_msg.name.push_back("joint_wheel_front_right");
      _joint_state_msg.name.push_back("joint_wheel_back_right");
      _joint_state_msg.name.push_back("joint_wheel_back_left");
      n_joints += 4;

      if(_enable_lift_control)
      {
         // TODO
      }
      else
      {
         _joint_state_msg.position.resize(n_joints);
         _joint_state_msg.velocity.resize(n_joints);
         // MMA FEATURE: get effort from motorcontrollers?

         // not implemented atm
         //_joint_state_msg.effort.resize(n_joints);
      }
   }

   // setup connections
   _sub_cmd_vel = _nh.subscribe<geometry_msgs::Twist>(topic_sub_cmd_vel, 1, &BaseControllerROS::cbCmdVel, this);
   _pub_odom = _nh.advertise<nav_msgs::Odometry>(topic_pub_odom, 1);
   _pub_enable_signal_off = _nh.advertise<std_msgs::Bool>(topic_pub_enable_signal_off, 1);

   // enable lift if necessary
   if(_enable_lift_control)
   {
      _sub_cmd_lift = _nh.subscribe<std_msgs::Int8>(topic_sub_cmd_lift, 1, &BaseControllerROS::cbCmdLift, this);

      const unsigned int num_lift = _lift_controller.getPositionVec().size();
      for(auto idx = 0u; idx < num_lift; idx++)
      {
         ros::Publisher position_pub = _nh.advertise<std_msgs::Float32>("lift/" + std::to_string(idx) + "/position/", 1);
         _pub_lift_pos_vec.push_back(position_pub);
      }
   }

   // finish flag
   evo::log::get() << _logger_prefix << "finished init process!" << evo::info;
   _is_initialized = true;
}

std::vector<MotorShieldConfig> BaseControllerROS::loadConfigROS(ros::NodeHandle& privateNh)
{
   std::vector<MotorShieldConfig> mc_config_ros;
   std::string paramName, paramPrefix;
   int motorshield_id        = 1;
   static const int n_motors = 2;

   std::map<std::string, double> param_map;

   int init_n_shields = 0;
   privateNh.param("init_n_motorshields", init_n_shields, 2);
   evo::log::get() << _logger_prefix << "Loading config for " 
                   << init_n_shields << " motorshields" <<
   evo::info;

   // check if the next motorshield exists
   paramPrefix = "ms" + std::to_string(motorshield_id);
   while(privateNh.hasParam(paramPrefix + "/enable"))
   {
      // error prevention
      if(motorshield_id > init_n_shields)
      {
         evo::log::get() << _logger_prefix
                         << "param server contains enable for next ms-id"
                         << ", but n-ms > init-n-ms! (" << motorshield_id << " > "
                         << init_n_shields << ")" << evo::warn;
         evo::log::get() << _logger_prefix
                         << "do you want to include the next shield? (y/n)" << evo::warn;
         char input;
         std::cin >> input; 
         try
         {
            if(std::tolower(input) != 'y')
            {
               evo::log::get() << _logger_prefix << "--> NO" << evo::info;
               break;
            }
            evo::log::get() << _logger_prefix << "--> YES" << evo::info;
         } 
         catch(const std::exception& e)
         {
            evo::log::get() << e.what() << evo::error;
         }
      }

      bool enable_mc = false;
      privateNh.getParam(paramPrefix + "/enable", enable_mc);
      evo::log::get() << _logger_prefix << "Enable ms" << motorshield_id << ": "
                      << enable_mc << evo::info;

      if(enable_mc)
      {
         evo::log::get() << _logger_prefix << "--------------------------"
                         << evo::info;
         evo::log::get() << _logger_prefix << "Loading parameters for ms"
                         << motorshield_id << evo::info;

         // load mc param
         MotorShieldConfig ms_config;
         ms_config.id   = motorshield_id;
         int timeout_ms = 10;
         if(!privateNh.getParam(paramPrefix + "/timeout_ms", timeout_ms))
         {
            timeout_ms = 10;
            evo::log::get() << _logger_prefix
                            << "no timeout_ms parameter given! using default: "
                            << ms_config.timeout_ms << evo::warn;
         }
         ms_config.timeout_ms = static_cast<uint32_t>(timeout_ms);
         // could also be loaded as param
         ms_config.n_motors = n_motors;

         // load params for two motors
         ms_config.motor_configs.clear();
         for(int motor_id = 0; motor_id < ms_config.n_motors; motor_id++)
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
            param_map["rpm_limit"]     = 0.0;
            param_map["gear_ratio"]    = 0.0;
            param_map["encoder_res"]   = 0.0;
            param_map["adc_conv"]      = 0.0;
            param_map["adc_offs"]      = 0.0;
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
            motor_config.kp                   = param_map.at("kp");
            motor_config.ki                   = param_map.at("ki");
            motor_config.kd                   = param_map.at("kd");
            motor_config.encoder_res          = param_map.at("encoder_res");
            motor_config.gear_ratio           = param_map.at("gear_ratio");
            motor_config.pwm_limit            = param_map.at("pwm_limit");
            motor_config.rpm_limit            = param_map.at("rpm_limit");
            motor_config.adc_conv_mm_per_tick = param_map.at("adc_conv");
            motor_config.adc_offs_mm          = param_map.at("adc_offs");

            motor_config.motor_mapping =
                static_cast<uint8_t>(param_map.at("motor_mapping"));
            ms_config.motor_configs.push_back(motor_config);
         }
         mc_config_ros.push_back(ms_config);
      }
      ++motorshield_id;
      paramPrefix = "ms" + std::to_string(motorshield_id);
   }
   return mc_config_ros;
}

void BaseControllerROS::publishOdomMsg(const MecanumVel& odom_vel,
                                       const MecanumPose& odom_pose)
{
   // create odom nav msg
   nav_msgs::Odometry odom;

   // header
   odom.header.stamp    = ros::Time::now();
   odom.header.frame_id = _odom_frame_id;
   odom.child_frame_id  = _odom_child_frame_id;

   // pose
   odom.pose.pose.position.x = odom_pose._x_m;
   odom.pose.pose.position.y = odom_pose._y_m;

   // MMA ERROR: should we really use this function?
   // MMA FEATURE: if we extend the functionality to lift and tilting, we have to
   // change this anyways
   tf::Quaternion pose_quaterion = tf::createQuaternionFromYaw(odom_pose._yaw_rad);
   odom.pose.pose.orientation.w  = pose_quaterion.getW();
   odom.pose.pose.orientation.y  = pose_quaterion.getY();
   odom.pose.pose.orientation.z  = pose_quaterion.getZ();
   odom.pose.pose.orientation.x  = pose_quaterion.getX();

   // twist
   odom.twist.twist.linear.x  = odom_vel._x_ms;
   odom.twist.twist.linear.y  = odom_vel._y_ms;
   odom.twist.twist.angular.z = odom_vel._yaw_rads;

   // covariances
   const double cpx   = _mecanum_covariance.cov_pos_x;
   const double cpy   = _mecanum_covariance.cov_pos_y;
   const double cpyaw = _mecanum_covariance.cov_pos_yaw;
   const double cvx   = _mecanum_covariance.cov_vel_x;
   const double cvy   = _mecanum_covariance.cov_vel_y;
   const double cvyaw = _mecanum_covariance.cov_vel_yaw;

   odom.twist.covariance = {cpx, 0.0, 0.0, 0.0, 0.0, 0.0,   
                            0.0, cpy, 0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, cpyaw, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, cvx, 0.0, 0.0,   
                            0.0, 0.0, 0.0, 0.0, cvy, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 0.0, cvyaw};

   odom.pose.covariance = odom.twist.covariance;

   _pub_odom.publish(odom);
}

void BaseControllerROS::publishOdomTF(const MecanumPose& odom_pose)
{
   tf::StampedTransform tf_odom;
   // header
   tf_odom.stamp_          = ros::Time::now();
   tf_odom.frame_id_       = _odom_frame_id;
   tf_odom.child_frame_id_ = _odom_child_frame_id;

   // position
   tf_odom.setOrigin(tf::Vector3(_odom_pose._x_m, _odom_pose._y_m, 0));

   // rotation
   tf::Quaternion pose_quaterion = tf::createQuaternionFromYaw(_odom_pose._yaw_rad);
   tf_odom.setRotation(pose_quaterion);

   _tf_pub_odom.sendTransform(tf_odom);
}

void BaseControllerROS::publishJointStates(const MecanumWheelData& wheel_positions,
                                           const MecanumWheelData& wheel_rotations)
{
   _joint_state_msg.position[0] = wheel_positions.front_left;
   _joint_state_msg.position[1] = wheel_positions.front_right;
   _joint_state_msg.position[2] = wheel_positions.back_right;
   _joint_state_msg.position[3] = wheel_positions.back_left;

   _joint_state_msg.velocity[0] = wheel_rotations.front_left;
   _joint_state_msg.velocity[1] = wheel_rotations.front_right;
   _joint_state_msg.velocity[2] = wheel_rotations.back_right;
   _joint_state_msg.velocity[3] = wheel_rotations.back_left;

   if(_enable_lift_control)
   {
      // TODO
   }

   _joint_state_msg.header.stamp = ros::Time::now();

   _pub_joint_state.publish(_joint_state_msg);
}

void BaseControllerROS::publishBaseStatus()
{
   // get drive data
   MecanumVel odom_vel;
   MecanumPose odom_pose_increment;
   MecanumWheelData wheel_positions;
   MecanumWheelData wheel_velocities;
   _mecanum_drive.getOdomComplete(odom_vel, odom_pose_increment, 
                                 wheel_positions, wheel_velocities);

   // update pose
   _odom_pose.updatePoseFromIncrement(odom_pose_increment);
   publishOdomMsg(odom_vel, _odom_pose);

   // eventually publish odom TF
   if(_enable_odom_tf)
   {publishOdomTF(_odom_pose);}

   // eventually publish joint states
   if(_enable_joint_state_publisher)
   {
      publishJointStates(wheel_positions, wheel_velocities);
   }

   // TODO: lift?
}

void BaseControllerROS::cbCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
   _stamp_cmd_vel     = ros::Time::now();
   _cmd_vel._x_ms     = cmd_vel->linear.x;
   _cmd_vel._y_ms     = cmd_vel->linear.y;
   _cmd_vel._yaw_rads = cmd_vel->angular.z;
}

void BaseControllerROS::publishLiftPos()
{
   const std::vector<float> positions = _lift_controller.getPositionVec();
   unsigned int idx                   = 0u;
   for(auto& pos : positions)
   {
      std_msgs::Float32 data;
      data.data = pos;
      _pub_lift_pos_vec[idx++].publish(data);
   }
   std::cout << std::endl;
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
      
      const MecanumVel zero_vel;
      _cmd_vel = zero_vel;
   }

   // only set cmd_vel if lift is not active
   if(!_lift_active || !_enable_lift_control)
   {
      _mecanum_drive.setCmdVel(_cmd_vel);
   }
}

void BaseControllerROS::checkAndApplyCmdLift()
{
   /*  REV
    *             +++ 1. What does "_strd" mean? Why not "_old" or something similar? -> @TODO @MBA stored
    *      INFO:  +++ 2. Why try exactly 2 times to reenable the drive motors?
    *             +++ 3. Why is there no error message after the second attempt, 
    *                considering that there will be no next attempt without another edge? (checkStatus() will reenable them?)
    *             -> @TODO @MBA
    *             MBA: Changed complete code structure
    */
   
   // check timestamp
   if(ros::Time::now().toSec() > (_stamp_cmd_lift.toSec() + _timeout_cmd_lift))
   {
      evo::log::get() << _logger_prefix
                      << "cmd lift timeout detected! stopping lift mechanism.."
                      << evo::warn;
      
      // Stop lift movement
      _cmd_lift = 0;
   }

   const bool lift_movement_req  = (_cmd_lift != 0);
   const bool drive_movement_req =  (_cmd_vel._x_ms != 0.0 || 
                                     _cmd_vel._y_ms != 0.0 || 
                                     _cmd_vel._yaw_rads != 0.0);

   // State transition: Lift inactive to lift active
   if(!_lift_active && lift_movement_req)
   {
      evo::log::get() << _logger_prefix 
                      << "disable drives | enable lift" 
                      << evo::info;

      // Try to disable all drive motors to avoid blockage of lift mechanism
      // Stopp movement
      const MecanumVel zero_vel;
      _mecanum_drive.setCmdVel(zero_vel);

      // Disable all drives motors to avoid blockage of the lift mechanism
      if(_motor_handler.disableAllDriveMotors())
      {
         // Lift is now active
         _lift_active = true;
      }
      else
      {
         // Disabling of drive motors failed
         // Lift is not active
         _lift_active = false;
      }
   }
   // State transition: Lift active to lift inactive
   // To avoid disabling the lift to fast check for a drive movement
   // request
   else if(_lift_active && !lift_movement_req && drive_movement_req)
   {
      evo::log::get() << _logger_prefix 
                      << "enable drives | disable lift" 
                      << evo::info;

      // Stop lift movement
      _lift_controller.setMovingDirection(0);

      // Reenable drive motors
      if(_motor_handler.enableAllDriveMotors())
      {
         // Lift movement is disabled
         _lift_active = false;
      }
      else
      {
         // Failed to enable drive motors
         // Lift mechanism is still active
         _lift_active = true;
      }
   }
   // Lift mechanism is active
   else if(_lift_active)
   {
      _lift_controller.setMovingDirection(_cmd_lift);
   }
   // Unknown state -> disable lift movement
   else
   {
      _lift_controller.setMovingDirection(0);
   }
}

const bool BaseControllerROS::checkStatus()
{
   /*  REV
    *  WHA STYLE: +++ It is not quite clear whether disabling all motors sets the handler to a valid status, 
    *             therefore leading to the else branch. -> TODO MBA clean up code
    *      INFO:  +++ As we only detect an error and create the edge ourselves by toggling the bool,
    *             I personally wouldn't quite call this edge detection. -> See below
    *  MPP INFO:  +++ Expression "falling edge detection" makes no sense. -> TODO MBA: better name instead of "falling edge detection"
    *  MBA: Changed complete code structure
    */

   // Check drive status
   const bool motor_status = _motor_handler.checkStatus();

   // State transition: Error to no error
   if(motor_status && _error_present)
   {
      // Try to enable all motors
      if(_motor_handler.enableAllMotors())
      {
         // Error healed
         _error_present = false;
      }
      else
      {
         // Not able to re-enable drives
         // error still present
         _error_present = true;
      }
   }
   // State transition: No error to error
   else if(!motor_status && !_error_present)
   {
      // Try to disable all motors
      if(_motor_handler.disableAllMotors())
      {
         // Error active
         _error_present = true;
      }
      else
      {
         // Unable to disable all drives
         // Error is present but not active
         _error_present = false;
      }
   }

   if(_error_present)
   {
      evo::log::get()
          << _logger_prefix
          << " motorshield communication error! -> Missing enable signal?"
          << evo::warn;
   }

   
   std_msgs::Bool enable_signal_off;
   enable_signal_off.data = _error_present;
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
         publishBaseStatus();
         if(_enable_lift_control)
         {publishLiftPos();}

         if(checkStatus())
         {
            checkAndApplyCmdVel();
            if(_enable_lift_control)
            {checkAndApplyCmdLift();}
         }

         _loop_rate_hz.sleep();
      }
   }
}

} // namespace evo
