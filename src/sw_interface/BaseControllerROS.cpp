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

/* REV
 * MPP INFO: * Does this header need to be public?
 *           * If yes, the directory sw_interface diverges from the standard ROS
 *             package layout.
 *             See: http://wiki.ros.org/Packages
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
    _logger_prefix("BaseControllerROS: "), 
    _lift_moving(false),
    _lift_moving_strd(false), 
    _error_present(false), 
    _is_initialized(false),
    _loop_rate_hz(50)
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
bool BaseControllerROS::init()
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

      /* REV
       * MPP INFO: * Prefer prefix increment operators over their postfix versions.
       *           * Suggestion
       *             Declaration of position_pub potentially superfluous. Use _nh.advertise() directly as argument
       *             for push_back(). Avoids a copy operation.
       */
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
         /*  REV
          *  WHA STYLE: n_motors is used one single time: to initialise a config value. 
          *             This way, "magic numbers" in the function body are avoided, which is good.
          *             Using this style of initialisation more often would improve maintainability and readability.
          */
         ms_config.n_motors = n_motors;

               // load params for two motors
         /* REV
          * MPP INFO: * for loop condition is a signed unsigned comparission.
          *           * Suggestion to simplify motor configuration retrieval
          *             Setup the map layout while defining the param_map variable
          *             with the help of an initializer list:
          * 
          *               std::map<std::string, double> param_map({
          *                 {"type",      0.},
          *                 {"ctrl_mode", 0.},
          *                 ...});
          * 
          *             Use the param() function with a default value to retrieve
          *             the parameters:
          * 
          *               if(privateNh.param(paramName, param.second, 0.))
          *               {
          *                 ...
          * 
          *             With that you can drop the parameter existence check with hasParam()
          *             and you do not need to reset the values of param_map in every loop
          *             cycle.
          *           * An unordered_map may be a better choice here since none of the
          *             advantages of map are used.
          * 
          * MMA: i want to print something which is also what causes the code to blow up
          */

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
   tf::Quaternion pose_quaternion = tf::createQuaternionFromYaw(odom_pose._yaw_rad);
   odom.pose.pose.orientation.w  = pose_quaternion.getW();
   odom.pose.pose.orientation.y  = pose_quaternion.getY();
   odom.pose.pose.orientation.z  = pose_quaternion.getZ();
   odom.pose.pose.orientation.x  = pose_quaternion.getX();

   // twist
   odom.twist.twist.linear.x  = odom_vel._x_ms;
   odom.twist.twist.linear.y  = odom_vel._y_ms;
   odom.twist.twist.angular.z = odom_vel._yaw_rads;

   // covariances
   /* REV
    * MPP INFO: const qualifiers pointless.
    */
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
   /* REV
    * MPP INFO: Suggestion
    *           Use an index based loop instead of a range base one. This would
    *           make things bit clearer since you are iterating over two std::vectors
    *           in the same way.
    */
   for(auto& pos : positions)
   {
      std_msgs::Float32 data;
      data.data = pos;
      _pub_lift_pos_vec[idx++].publish(data);
   }
   /* REV
    * MPP STYLE: Output really needed? Slows down the main processing loop.
    *            Especially the flushing part of std::endl.
    */
   std::cout << std::endl;
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
      _cmd_vel = MecanumVel();
      return;
   }

   if(ros::Time::now().toSec() > (_stamp_cmd_vel.toSec() + _timeout_cmd_vel))
   {
      evo::log::get() << _logger_prefix
                      << "cmd vel timeout detected! stopping robot.." << evo::warn;
      MecanumVel zero;
      _mecanum_drive.setCmdVel(zero);
   }
   else
   {
      _mecanum_drive.setCmdVel(_cmd_vel);
   }
}

/* REV
 * MPP STYLE: Member variable _lift_moving_strd only used in this function.
 *            Suggestion: Turn it into static local variable.
 */
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
      /* REV
       * MPP INFO: Shorter: _lift_moving = (_cmd_lift != 0);
       */
      if(_cmd_lift != 0)
      {
         _lift_moving = true;
      }
      else
      {
         _lift_moving = false;
      }

      _lift_controller.setMovingDirection(_cmd_lift);
   }


   /*  REV
    *  WHA STYLE: As _lift_moving_strd seems to be the old _lift_moving value, this should be an edge detection.
    *             1. What does "_strd" mean? Why not "_old" or something similar?
    *      INFO:  2. Why try exactly 2 times to reenable the drive motors?
    *             3. Why is there no error message after the second attempt,
    *                considering that there will be no next attempt without another edge? (checkStatus() will reenable them?)
    *             4. Why not set the speed to zero first, then enable the drives?
    *             5. For future code formatting: Format change commits should probably not be mixed with "real" commits
    */

   if(_lift_moving && !_lift_moving_strd)
   {
      MecanumVel zero;
      _mecanum_drive.setCmdVel(zero);

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

      _mecanum_drive.setCmdVel(zero);
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

/* REV
 * MPP STYLE: Member variable _error_present only used in this function.
 *            Suggestion: Turn it into a (static) local variable.
 * MPP INFO: Suggestion
 *           Assign the value of _error_present to enable_signal_off.data
 *           right before publishing the message. Spares you an assignment
 *           and makes things a bit clearer.
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
    *  WHA STYLE: It is not quite clear whether disabling all motors sets the handler to a valid status, 
    *             therefore leading to the else branch.
    *      INFO:  As we only detect an error and create the edge ourselves by toggling the bool,
    *             I personally wouldn't quite call this edge detection.
    *  MPP INFO: * Why wait an entire loop cycle before handling the error? 
    *            * Expression "falling edge detection" makes no sense.
    */
   // falling edge detection for error status to re-enable motors
   else if(_error_present)
   {
      _error_present = false;

      /*  REV
       *  WHA STYLE: If there is a comment admitting that this is a hack, I would like to know more 
       *             about why this is a hack and why this hack is necessary.
       *  MPP INFO: Sleep disrupts main loop. Program will not react to anything during this period.
       *            Is this an acceptable behavior?
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
         publishBaseStatus();

         // MMA: replace in base status?
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
