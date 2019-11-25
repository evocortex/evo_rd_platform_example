//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file DCDCControllerROS.h
 * @author evocortex (MBA) (info@evocortex.com)
 *
 * @brief ROS controller for the DC/DC Shield
 *
 * @version 0.1
 * @date 2019-11-21
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#ifndef DCDCCONTROLLER_ROS_H_
#define DCDCCONTROLLER_ROS_H_

// ROS header
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// DC/DC HW Header
#include <evo_dcdc_shield_interface/DCDCShield.h>
#include <evo_mbed/tools/com/ComServer.h>

// Logger
#include <evo_logger/log/Logger.h>

namespace evo {

class DCDCControllerROS
{
 private:
   void cbELMLedBrightness(const std_msgs::Float32::ConstPtr& data);

   std::string _logger_prefix;

   std::shared_ptr<evo_mbed::ComServer> _can_interface;
   std::shared_ptr<evo_mbed::DCDCShield> _dc_shield;

   bool _elm_present                    = false;
   const unsigned int _elm_timeout_msec = 500u;
   ros::Time _elm_led_brightness_upd_timestamp;
   float _elm_led_brightness = 0.0f;

   ros::NodeHandle _nh;

   ros::Publisher _pub_battery_voltage;
   ros::Subscriber _sub_elm_led_brightness;

   ros::Rate _loop_rate_hz;

   bool _is_initialized = false;

 public:
   DCDCControllerROS();

   bool init();

   void main_loop();
};

} // namespace evo
#endif // DCDCCONTROLLER_ROS_H_
