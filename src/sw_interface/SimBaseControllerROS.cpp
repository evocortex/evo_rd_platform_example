#include "sw_interface/SimBaseControllerROS.h"
namespace evo {

SimBaseControllerROS::SimBaseControllerROS() :
   _logger_prefix("SimBaseControllerROS: "), 
   _loop_rate_hz(50)
{
   evo::log::init("");

   ros::NodeHandle privateNh("~");

   // parameters for the mecanum drive
   double wheel_radius_in_m, wheel_distance_front_back_in_m,
       wheel_distance_left_right_in_m;
   privateNh.param("wheel_radius_in_m", wheel_radius_in_m, 0.0);
   privateNh.param("wheel_distance_front_back_in_m", wheel_distance_front_back_in_m, 0.0);
   privateNh.param("wheel_distance_left_right_in_m", wheel_distance_left_right_in_m, 0.0);
   _mecanum_drive.setWheelRadiusInM(wheel_radius_in_m);
   _mecanum_drive.setWheelDistanceFrontBackInM(wheel_distance_front_back_in_m);
   _mecanum_drive.setWheelDistanceLeftRightInM(wheel_distance_left_right_in_m);

   // covariances
   privateNh.param("covariance_pos_x", _mecanum_covariance.cov_pos_x, 1.0);
   privateNh.param("covariance_pos_y", _mecanum_covariance.cov_pos_y, 1.0);
   privateNh.param("covariance_pos_yaw", _mecanum_covariance.cov_pos_yaw, 1.0);
   privateNh.param("covariance_vel_x", _mecanum_covariance.cov_vel_x, 1.0);
   privateNh.param("covariance_vel_y", _mecanum_covariance.cov_vel_y, 1.0);
   privateNh.param("covariance_vel_yaw", _mecanum_covariance.cov_vel_yaw, 1.0);

   // ROS 
   double loop_rate_hz;
   privateNh.param("loop_rate_hz", loop_rate_hz, 50.0);
   _loop_rate_hz = ros::Rate(loop_rate_hz);

   std::string topic_sub_cmd_vel, topic_pub_odom;
   privateNh.param("topic_pub_odom", topic_pub_odom, std::string("odom"));
   privateNh.param("topic_sub_cmd_vel", topic_sub_cmd_vel, std::string("cmd_vel"));
   privateNh.param("cmd_vel_timeout_s", _timeout_cmd_vel, 0.1);

    // odometry
   privateNh.param("enable_odom_tf", _enable_odom_tf, true);
   privateNh.param("odom_frame_id", _odom_frame_id, std::string("odom"));
   privateNh.param("odom_child_frame_id", _odom_child_frame_id, std::string("base_footprint"));

   // setup connections
   _sub_cmd_vel = _nh.subscribe<geometry_msgs::Twist>(topic_sub_cmd_vel, 1, &SimBaseControllerROS::cbCmdVel, this);
   _pub_odom = _nh.advertise<nav_msgs::Odometry>(topic_pub_odom, 1);


   // setup the topic to motor mapping
   std::string topic_pub_front_left, topic_pub_back_left, topic_pub_front_right, topic_pub_back_right;

   privateNh.param("topic_pub_front_left", topic_pub_front_left, std::string("/evo_robot/wheel_fl_controller/command"));
   privateNh.param("topic_pub_back_left", topic_pub_back_left, std::string("/evo_robot/wheel_bl_controller/command"));
   privateNh.param("topic_pub_front_right", topic_pub_front_right, std::string("/evo_robot/wheel_fr_controller/command"));
   privateNh.param("topic_pub_back_right", topic_pub_back_right, std::string("/evo_robot/wheel_br_controller/command"));


   _map_pub_sim_wheeldata[MOTOR_MAPPING_MECANUM::FRONT_LEFT] = _nh.advertise<std_msgs::Float64>(topic_pub_front_left, 1);
   _map_pub_sim_wheeldata[MOTOR_MAPPING_MECANUM::BACK_LEFT] = _nh.advertise<std_msgs::Float64>(topic_pub_back_left, 1);
   _map_pub_sim_wheeldata[MOTOR_MAPPING_MECANUM::FRONT_RIGHT] = _nh.advertise<std_msgs::Float64>(topic_pub_front_right, 1);
   _map_pub_sim_wheeldata[MOTOR_MAPPING_MECANUM::BACK_RIGHT] = _nh.advertise<std_msgs::Float64>(topic_pub_back_right, 1);


   // setup the joint names for more easy access later
   std::string joint_name_front_left, joint_name_back_left, joint_name_front_right, joint_name_back_right, topic_sub_jointstate;
   privateNh.param("joint_name_front_left", joint_name_front_left, std::string("fl_lift_link_2_fl_rim_link"));
   privateNh.param("joint_name_back_left", joint_name_back_left, std::string("fl_lift_link_2_bl_rim_link"));
   privateNh.param("joint_name_front_right", joint_name_front_right, std::string("fl_lift_link_2_fr_rim_link"));
   privateNh.param("joint_name_back_right", joint_name_back_right, std::string("fl_lift_link_2_br_rim_link"));

   _map_name_to_pos[MOTOR_MAPPING_MECANUM::FRONT_LEFT] = joint_name_front_left;
   _map_name_to_pos[MOTOR_MAPPING_MECANUM::BACK_LEFT] = joint_name_back_left;
   _map_name_to_pos[MOTOR_MAPPING_MECANUM::FRONT_RIGHT] = joint_name_front_right;
   _map_name_to_pos[MOTOR_MAPPING_MECANUM::BACK_RIGHT] = joint_name_back_right;

   privateNh.param("topic_sub_jointstate", topic_sub_jointstate, std::string("/evo_robot/joint_states"));
   _sub_sim_wheeldata = _nh.subscribe<sensor_msgs::JointState>(topic_sub_jointstate, 1, &SimBaseControllerROS::cbJointState, this);
}

SimBaseControllerROS::~SimBaseControllerROS() 
{

}

void SimBaseControllerROS::publishOdomMsg(const MecanumVel& odom_vel, const MecanumPose& odom_pose)
{
   // create odom nav msg
   nav_msgs::Odometry odom;

   // header
   odom.header.stamp    = ros::Time::now();
   odom.header.frame_id = _odom_frame_id;
   odom.child_frame_id  = _odom_child_frame_id;

   // pose
   odom.pose.pose.position.x     = odom_pose._x_m;
   odom.pose.pose.position.y     = odom_pose._y_m;
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

void SimBaseControllerROS::publishOdomTF(const MecanumPose& odom_pose)
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

void SimBaseControllerROS::cbJointState(const sensor_msgs::JointState::ConstPtr& jointstate)
{
   // first get the values from the joint state
   MecanumWheelData wheel_positions;
   MecanumWheelData wheel_velocities;
   
   int i = 0;
   for(auto& name : jointstate->name)
   {
      for(auto& saved_name : _map_name_to_pos)
      {
         if(saved_name.second == name)
         {
            switch(saved_name.first)
            {
               case MOTOR_MAPPING_MECANUM::FRONT_LEFT:
               {
                  wheel_positions.front_left = jointstate->position.at(i);
                  wheel_velocities.front_left = jointstate->velocity.at(i);
               }
               break;
               case MOTOR_MAPPING_MECANUM::BACK_LEFT:
               {
                  wheel_positions.back_left = jointstate->position.at(i);
                  wheel_velocities.back_left = jointstate->velocity.at(i);
               }
               break;
               case MOTOR_MAPPING_MECANUM::FRONT_RIGHT:
               {
                  wheel_positions.front_right = jointstate->position.at(i);
                  wheel_velocities.front_right = jointstate->velocity.at(i);
               }
               break;
               case MOTOR_MAPPING_MECANUM::BACK_RIGHT:
               {
                  wheel_positions.back_right = jointstate->position.at(i);
                  wheel_velocities.back_right = jointstate->velocity.at(i);
               }
               default:
               {break;}
            }
         }
      }
      i++;
   }

   // proceed to calculate odom
   MecanumVel odom_vel;
   MecanumPose odom_pose_increment;

   _mecanum_drive.wheelData2OdomVel(wheel_velocities, odom_vel);

   _mecanum_drive.wheelData2OdomPoseInc(wheel_positions, _last_wheel_positions, odom_pose_increment);

   // update pose
   _odom_pose.updatePoseFromIncrement(odom_pose_increment);
   publishOdomMsg(odom_vel, _odom_pose);

   // eventually publish odom TF
   if(_enable_odom_tf)
      {publishOdomTF(_odom_pose);}
}


void SimBaseControllerROS::cbCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
   _stamp_cmd_vel     = ros::Time::now();
   _cmd_vel._x_ms     = cmd_vel->linear.x;
   _cmd_vel._y_ms     = cmd_vel->linear.y;
   _cmd_vel._yaw_rads = cmd_vel->angular.z;
}

void SimBaseControllerROS::checkAndApplyCmdVel()
{
   MecanumWheelData wd;

   // check timestamp
   if(ros::Time::now().toSec() > (_stamp_cmd_vel.toSec() + _timeout_cmd_vel))
   {
      evo::log::get() << _logger_prefix << "cmd vel timeout detected!" << 
                     " stopping robot.." << evo::warn;
      MecanumVel zero;
      _mecanum_drive.cmdVel2wheelData(zero, wd);
   }
   else
   {
      _mecanum_drive.cmdVel2wheelData(_cmd_vel, wd);
   }

   std_msgs::Float64 wheel_speed;
   wheel_speed.data = wd.front_left;
   _map_pub_sim_wheeldata[MOTOR_MAPPING_MECANUM::FRONT_LEFT].publish(wheel_speed); 

    wheel_speed.data = wd.back_left;
   _map_pub_sim_wheeldata[MOTOR_MAPPING_MECANUM::BACK_LEFT].publish(wheel_speed); 

    wheel_speed.data = wd.front_right;
   _map_pub_sim_wheeldata[MOTOR_MAPPING_MECANUM::FRONT_RIGHT].publish(wheel_speed); 

    wheel_speed.data = wd.back_right;
   _map_pub_sim_wheeldata[MOTOR_MAPPING_MECANUM::BACK_RIGHT].publish(wheel_speed); 
}

void SimBaseControllerROS::main_loop()
{
   ROS_INFO_STREAM(_logger_prefix << "Starting main control loop!..");

   while(ros::ok())
   {
      ros::spinOnce();
      checkAndApplyCmdVel();
      _loop_rate_hz.sleep();
   }
}



} // namespace evo
