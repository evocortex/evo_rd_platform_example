//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file ToFControllerROS.cpp
 * @author evocortex (info@evocortex.com)
 *
 * @brief ROS controller for the ToF sensors
 *
 * @version 0.1
 * @date 2019-09-26
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#include "sw_interface/DCDCControllerROS.h"

namespace evo {
DCDCControllerROS::DCDCControllerROS() :
    _is_initialized(false), _loop_rate_hz(10), _logger_prefix("DCDCControllerROS: ")
{}

bool DCDCControllerROS::init()
{
   evo::log::get() << _logger_prefix << "start init process!" << evo::info;

   // load all relevant parameters
   ros::NodeHandle privateNh("~");
   // parameters for the can interface
   std::string can_interface_name;
   privateNh.param("can_interface_name", can_interface_name,
                   std::string("can_motor"));
   _can_interface = std::make_shared<evo_mbed::ComServer>();
   if(_can_interface->init(can_interface_name) != evo_mbed::RES_OK)
   {
      evo::log::get() << _logger_prefix
                      << "initialization of the CAN interface failed!" << evo::error;
      evo::log::get() << _logger_prefix << "Check if the interfacename ["
                      << can_interface_name << "] is correct!" << evo::error;
      evo::log::get() << _logger_prefix << "--> Shutdown" << evo::error;
      exit(1);
   }

   int node_id = 10;
   privateNh.param("node_id", node_id, 10);

   if(node_id < 1 || node_id > 127)
   {
      evo::log::get() << _logger_prefix << "node ID '" << +node_id
                      << "' is invalid [1;127] is allowed" << evo::error;
   }

   if(node_id < 10 || node_id > 20)
   {
      evo::log::get() << _logger_prefix << "node ID '" << +node_id
                      << "' DC/DC shield id is normally [10;20]" << evo::warn;
   }

   // init DC/DC shield
   if(!_dc_shield)
   {
      _dc_shield = std::make_shared<evo_mbed::DCDCShield>(node_id, _can_interface);
   }

   if(!_dc_shield->init())
   {
      evo::log::get() << _logger_prefix << " Failed to initialize DC/DC shield!"
                      << evo::error;
      exit(1);
   }

   if(!_dc_shield->setComTimeoutMS(_elm_timeout_msec))
   {
      evo::log::get() << _logger_prefix << " Failed to set DC/DC shield timeout!"
                      << evo::error;
      exit(1);
   }

   // check if ELM is present
   int elm_present = 0;
   privateNh.param("elm_present", elm_present, 0);

   if(elm_present != 0)
   {
      evo::log::get() << _logger_prefix << " ELM present at robot!" << evo::info;
      _elm_present = true;
   }
   else
   {
      elm_present = false;
   }

   if(elm_present)
   {
      // Config PWM frequency for ELM
      if(!_dc_shield->setChn2PWMFrequencyHz(30000))
      {
         evo::log::get() << _logger_prefix << " Failed to set ELM-LED PWM frequency!"
                         << evo::error;
         exit(1);
      }

      if(!_dc_shield->setChn2Status(true))
      {
         evo::log::get() << _logger_prefix
                         << " Failed to enable ELM-LED PWM channel!" << evo::error;
         exit(1);
      }

      // elm subscriber
      _elm_led_brightness               = 0.0f;
      _elm_led_brightness_upd_timestamp = ros::Time::now();
      _sub_elm_led_brightness           = privateNh.subscribe<std_msgs::Float32>(
          "elm_led_brightness", 1, &DCDCControllerROS::cbELMLedBrightness, this);
   }

   // parameters for this class
   double loop_rate_hz;
   privateNh.param("loop_rate_hz", loop_rate_hz, 20.0);
   _loop_rate_hz = ros::Rate(loop_rate_hz);

   // battery voltage publisher
   _pub_battery_voltage =
       privateNh.advertise<std_msgs::Float32>("battery_voltage_vdc", 1);

   // finish flag
   evo::log::get() << _logger_prefix << "finished init process!" << evo::info;
   _is_initialized = true;
   return true;
}

void DCDCControllerROS::main_loop()
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

      while(ros::ok())
      {
         ros::spinOnce();

         // publish battery voltage
         std_msgs::Float32 f32;
         f32.data = _dc_shield->getBatteryVoltage();
         _pub_battery_voltage.publish(f32);

         if(_elm_present)
         {
            auto delta_time_sec =
                ros::Time::now() - _elm_led_brightness_upd_timestamp;
            if((delta_time_sec.toSec() * 1000.0) >
               static_cast<double>(_elm_timeout_msec))
            {
               evo::log::get()
                   << _logger_prefix << "ELM-LED brightness subscriber timeout"
                   << evo::warn;
               _elm_led_brightness = 0.0f;
            }

            // update ELM LED brightness
            if(!_dc_shield->setChn2DutyCycle(_elm_led_brightness))
            {
               evo::log::get()
                   << _logger_prefix
                   << "Failed to set ELM-LED brightness! (CAN-Bus Problem?)"
                   << evo::error;
            }
         }

         _loop_rate_hz.sleep();
      }
   }
}

void DCDCControllerROS::cbELMLedBrightness(const std_msgs::Float32::ConstPtr& data)
{
   _elm_led_brightness = data->data;

   // limit values
   if(_elm_led_brightness > 100.0f)
   {
      _elm_led_brightness = 100.0f;
   }
   else if(_elm_led_brightness < 0.0f)
   {
      _elm_led_brightness = 0.0f;
   }

   _elm_led_brightness_upd_timestamp = ros::Time::now();
}

} // namespace evo
