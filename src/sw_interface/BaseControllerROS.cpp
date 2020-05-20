//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file BaseControllerROS.cpp
 * //  REV
 * //  WHA STYLE: wrong author 
 * @author evocortex (info@evocortex.com)
 *
 * //  REV
 * //  WHA STYLE: Proposal: "Interface class for accessing CAN motor controls using ROS"
 * @brief Interface class to bring the CAN motor stuff into ROS
 *
 * @version 0.1
 * @date 2019-08-14
 * 
 * //  REV
 * //  WHA STYLE: Year of creation or range of years during which this file was edited?
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#include "sw_interface/BaseControllerROS.h"

namespace evo {

/*  REV
 *  WHA STYLE: In an initializer list, how many initialisations should there be per code line?
 *             One per line would improve readability.
 *  MPP STYLE: * Suggestion for initializer list
 * 
 *              BaseControllerROS::BaseControllerROS()
 *                : _logger_prefix("BaseControllerROS: ")
 *                , _lift_moving(false)
 *                , ...
 *              { ... }
 * 
 *              Advantages: Clearer and easier to rearrange
 *            * Initializer list does not include all member variables. Is everything
 *              correctly initialized?
 *            * Constructor could be declared noexcept.
 *  MPP ERROR: Members are not initialized in the order in which they are declared.
 */
BaseControllerROS::BaseControllerROS() :
    _logger_prefix("BaseControllerROS: ")
    ,_lift_moving(false)
    ,_lift_moving_strd(false)
    ,_error_present(false)
    ,_is_initialized(false)
    ,_loop_rate_hz(50)
{
   evo::log::init("");
}

/*  REV
 *  WHA STYLE: Proposal: Variable declarations at the top, initialisations afterwards
 *  +++ MPP ERROR: In case of an initialization failure this function behalves inconsistently.
 *               Can Interface: The program is terminated by force.
 *               Mecanum Drive: Function just quits without any feedback to client.
 *             Suggestion: Return a bool signifing whether init() was successfull and let the
 *                         client decide how to handle it.
 *  MPP STYLE: Member variable _nh is only used in this function.
 *             Suggestion: Use a local variable instead or use the private node handle to
 *                         establish the topics (The ROS root namespace can be referenced with a
 *                         preceeding /). The private node handle could be made a member variable
 *                         since it is used in other member function.
 *  +++ HEN INFO: Return success status or error code to give the caller the possibility to react
 */
void BaseControllerROS::init()
{
   evo::log::get() << _logger_prefix << "start init process!" << evo::info;

   bool success = true;
   // load all relevant parameters
   ros::NodeHandle privateNh("~");

   // parameters for the motor manager
   /* REV
    * review: matter of taste
    * +++ MPP INFO: Suggestion
    *           Make the string parameter retrieval a bit nicer:
    * 
    *             privateNh.param<std::string>("...", con_interface_name, "can-motor");
    */
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

      // TODO REV no exit() inside class
      exit(1);
   }
   _motor_handler.setConfig(loadConfigROS(privateNh));
   success &= _motor_handler.initFromConfig();
   _motor_handler.initMotorMapping(_mecanum_drive, _lift_controller);
   success &= _motor_handler.enableAllMotors();

   // parameters for the mecanum drive
   /* REV
    * MPP STYLE: Suggestion
    *            Use a new line for every declared variable. Makes things easier to read.
    *            Especially when they are initialized at the same time.
    * 
    *            double wheel_radius_in_m,
    *                   wheel_distance_front_back_in_m,
    *                   wheel_distance_left_right_in_m;
    */
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
   /* REV
    * MPP STYLE: Member variable _mecanum_inverted is only used here. Superfluous?
    */
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

   privateNh.param("enable_lift_control", _lift_control_enabled, false);

   // setup connections
   _sub_cmd_vel = _nh.subscribe<geometry_msgs::Twist>(topic_sub_cmd_vel, 1, &BaseControllerROS::cbCmdVel, this);
   _pub_odom = _nh.advertise<nav_msgs::Odometry>(topic_pub_odom, 1);
   _pub_enable_signal_off = _nh.advertise<std_msgs::Bool>(topic_pub_enable_signal_off, 1);

   // enable lift if necessary
   if(_lift_control_enabled)
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

/*  REV
 *  WHA STYLE: Why not in a single line? -> multiple lines make sense in some situations, but this should be consistent
 *  MPP STYLE: * Function could be declared const since it does not change member variables.
 *             * Inconsistent code style for variable names.
 */
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
      /*  REV
       *  WHA STYLE: Name consistency: enable_mc <-> enable_ms <-> timeout_ms <-> _x_ms
       */

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

         /*  REV
          *  WHA STYLE:    if(!privateNh.hasParam(..)){timeout_ms = 10;} else {privateNh.getParam(...)} would be FAR more readable
          *                see further below for an example
          *  MPP STYLE: Suggestion
          *             Use the param() function instead since it allows you to specify
          *             a default value for timeout_ms.
          */
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

         /*  REV
          *  WHA STYLE: n_motors is used one single time: to initialise a config value. 
          *             This way, "magic numbers" in the function body are avoided, which is good.
          *             Using this style of initialisation more often would improve maintainability and readability.
          */
         mc_config.n_motors = n_motors;
         mc_config.motor_configs.clear();

         // load params for two motors
         /* REV
          * MPP INFO: * +++ for loop condition is a signed unsigned comparission. -> TODO MMA
          */
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
            /* REV
             * MPP STYLE: Bounds checked element access with at() is pointless since we know
             *            that the keys exit. 
             *            Suggestion: Use operator[] instead.
             */
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
            motor_config.adc_conv_mm_per_tick = param_map.at("adc_conv");
            motor_config.adc_offs_mm          = param_map.at("adc_offs");

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

/* REV
 * MPP STYLE: * Member variable _odom_pose only used in this function.
 *              Suggestion: Turn it into a local variable.
 *            * Suggestion
 *              Turn the odom message into a member variable and set it up
 *              in the init() function. This would make the member variables
 *              _odom_frame_id, _odom_child_frame_id and _mecanum_covariance
 *              obsolete.
 */
void BaseControllerROS::publishOdom()
{
   MecanumVel odom_raw = _mecanum_drive.getOdom();

   nav_msgs::Odometry odom;
   odom.header.stamp          = ros::Time::now();
   odom.header.frame_id = _odom_frame_id;
   odom.child_frame_id = _odom_child_frame_id;

   odom.twist.twist.linear.x  = odom_raw._x_ms;
   odom.twist.twist.linear.y  = odom_raw._y_ms;
   odom.twist.twist.angular.z = odom_raw._yaw_rads;

   // add up to position - from vel
   //_odom_pose.updatePoseFromVel(odom_raw, _loop_rate_hz.cycleTime().toSec());

   // add up position - from increment
   _odom_pose.updatePoseFromIncrement(_mecanum_drive.getPoseIncrement());

   odom.pose.pose.position.x     = _odom_pose._x_m;
   odom.pose.pose.position.y     = _odom_pose._y_m;

   tf::Quaternion pose_quaternion = tf::createQuaternionFromYaw(_odom_pose._yaw_rad);
   odom.pose.pose.orientation.w   = pose_quaternion.getW();
   odom.pose.pose.orientation.y   = pose_quaternion.getY();
   odom.pose.pose.orientation.z   = pose_quaternion.getZ();
   odom.pose.pose.orientation.x   = pose_quaternion.getX();

   // covariances
   const double cpx   = _mecanum_covariance.cov_pos_x;
   const double cpy   = _mecanum_covariance.cov_pos_y;
   const double cpyaw = _mecanum_covariance.cov_pos_yaw;
   const double cvx   = _mecanum_covariance.cov_vel_x;
   const double cvy   = _mecanum_covariance.cov_vel_y;
   const double cvyaw = _mecanum_covariance.cov_vel_yaw;

   odom.twist.covariance = {cpx, 0.0, 0.0, 0.0, 0.0, 0.0,   0.0, cpy, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, cpyaw, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, cvx, 0.0, 0.0,   0.0, 0.0, 0.0,
                            0.0, cvy, 0.0, 0.0, 0.0, 0.0,   0.0, 0.0, cvyaw};

   odom.pose.covariance = odom.twist.covariance;

   _pub_odom.publish(odom);

   // Transform
   if(_enable_odom_tf)
   {
      tf::StampedTransform tf_odom;
      tf_odom.setOrigin(tf::Vector3(_odom_pose._x_m, _odom_pose._y_m, 0));
      tf_odom.setRotation(pose_quaternion);
      tf_odom.frame_id_       = _odom_frame_id;
      tf_odom.child_frame_id_ = _odom_child_frame_id;
      tf_odom.stamp_          = odom.header.stamp;
      _tf_pub_odom.sendTransform(tf_odom);
   }
}

void BaseControllerROS::cbCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
   _stamp_cmd_vel     = ros::Time::now();

   /*  REV
    *  +++ WHA INFO: To the left, there is x_ms; to the right, just x. What do "ms" and "rads" mean? 
    *            Velocity has the unit "meters per second", yet "ms" looks more like "milliseconds" or "meter times second".
    */
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
}

void BaseControllerROS::cbCmdLift(const std_msgs::Int8::ConstPtr& cmd_lift)
{
   _stamp_cmd_lift = ros::Time::now();
   _cmd_lift       = cmd_lift->data;
}

void BaseControllerROS::checkAndApplyCmdVel()
{
   // Check if lift drive is moving -> ignore commands
   if(_lift_moving)
   {
      _cmd_vel._x_ms     = 0.0;
      _cmd_vel._y_ms     = 0.0;
      _cmd_vel._yaw_rads = 0.0;
      return;
   }

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
      _lift_moving = false;
   }
   else
   {
      _lift_moving = _cmd_lift != 0;
      _lift_controller.setMovingDirection(_cmd_lift);
   }


   /*  REV
    *             +++ 1. What does "_strd" mean? Why not "_old" or something similar? -> @TODO @MBA stored
    *      INFO:  +++ 2. Why try exactly 2 times to reenable the drive motors?
    *             +++ 3. Why is there no error message after the second attempt, 
    *                considering that there will be no next attempt without another edge? (checkStatus() will reenable them?)
    *             -> @TODO @MBA
    */

   if(_lift_moving && !_lift_moving_strd)
   {
      MecanumVel zero;
      _mecanum_drive.setTargetSpeed(zero);

      _motor_handler.disableAllDriveMotors();
   }
   else if(!_lift_moving && _lift_moving_strd)
   {
      MecanumVel zero;

      if(!_motor_handler.enableAllDriveMotors())
      {
         evo::log::get() << _logger_prefix << "Failed to enable all drives retrying!"
                         << evo::warn;

         _motor_handler.enableAllDriveMotors();
      }

      _mecanum_drive.setTargetSpeed(zero);
   }

   _lift_moving_strd = _lift_moving;
}

/*  REV
 *  WHA STYLE: As we use git, there should be no need for function corpses remaining in the source code.
 */

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

   /*  REV
    *  WHA STYLE: +++ It is not quite clear whether disabling all motors sets the handler to a valid status, 
    *             therefore leading to the else branch. -> TODO MBA clean up code
    *      INFO:  +++ As we only detect an error and create the edge ourselves by toggling the bool,
    *             I personally wouldn't quite call this edge detection. -> See below
    *  MPP INFO:  +++ Expression "falling edge detection" makes no sense. -> TODO MBA: better name instead of "falling edge detection"
    */
   // falling edge detection for error status to re-enable motors
   else if(_error_present)
   {
      _error_present = false;

      /*  REV
       *  WHA STYLE: +++ If there is a comment admitting that this is a hack, I would like to know more 
       *             about why this is a hack and why this hack is necessary. -> TODO MBA
       */

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

         /*  REV
          *  +++ WHA STYLE: Not really a fan of using if-clauses like this.
          */
         if(_lift_control_enabled) { publishLiftPos(); }

         if(checkStatus())
         {
            checkAndApplyCmdVel();
            if(_lift_control_enabled) checkAndApplyCmdLift();
         }

         _loop_rate_hz.sleep();
      }
   }
}

} // namespace evo
