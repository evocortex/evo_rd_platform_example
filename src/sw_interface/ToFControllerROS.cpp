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

#include "sw_interface/ToFControllerROS.h"

namespace evo {
ToFControllerROS::ToFControllerROS() :
    _is_initialized(false), _loop_rate_hz(2), _logger_prefix("ToFControllerROS: ")
{}

bool ToFControllerROS::init()
{
   evo::log::get() << _logger_prefix << "start init process!" << evo::info;
   _map_ToF_boards.clear();

   bool success = true;
   // load all relevant parameters
   ros::NodeHandle privateNh("~");

   // parameters for the can interface
   std::string can_interface_name;
   privateNh.param("can_interface_name", can_interface_name,
                   std::string("can-motor"));
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

   // init ToF sensors
   int n_tof_sensor_boards = 0;
   privateNh.param("n_ToF_sensorboards", n_tof_sensor_boards, 0);
   evo::log::get() << _logger_prefix << "try to init " << n_tof_sensor_boards
                   << " ToF Boards.." << evo::info;

   int sensor_id;
   std::string prefix, sensor_frame_id;
   for(int i = 1; i <= n_tof_sensor_boards; i++)
   {
      prefix = "sensorboard" + std::to_string(i) + "/";
      privateNh.param(prefix + "id", sensor_id, 0);
      privateNh.param(prefix + "frame_id", sensor_frame_id, std::string("ToF_0"));

      std::shared_ptr<evo_mbed::ToFBoard> sensor_board =
          std::make_shared<evo_mbed::ToFBoard>(sensor_id, _can_interface);
      if(!sensor_board->init())
      {
         evo::log::get() << _logger_prefix << "sensor board " << i << " init failed!"
                         << evo::error;
         evo::log::get() << _logger_prefix << "params: sensor_id(" << sensor_id
                         << ") - frame_id(" << sensor_frame_id << ")" << evo::error;
         success = false;
      }
      else
      {
         _map_ToF_boards[sensor_frame_id]             = sensor_board;
         _map_ToF_sensors[sensor_frame_id + "_left"]  = sensor_board->getSensor(1);
         _map_ToF_sensors[sensor_frame_id + "_right"] = sensor_board->getSensor(0);
      }
   }

   if(!success)
   {
      evo::log::get()
          << _logger_prefix
          << "init process not successful! some sensors are not connected!"
          << evo::error;
      return false;
   }

   // 27° = 0,471239 rad
   privateNh.param("ToF_FoV_rad", _ToF_FoV_rad, 0.471239);
   privateNh.param("ToF_range_max_m", _ToF_range_max_m, 4.0);

   // parameters for this class
   double loop_rate_hz;
   privateNh.param("loop_rate_hz", loop_rate_hz, 20.0);
   _loop_rate_hz = ros::Rate(loop_rate_hz);

   std::string topic_pub_range, topic_pub_detailed;
   privateNh.param("topic_pub_range", topic_pub_range, std::string("evo_ToF/range"));
   privateNh.param("topic_pub_detailed", topic_pub_detailed,
                   std::string("evo_ToF/detailed"));

   // setup connections
   _pub_range            = _nh.advertise<sensor_msgs::Range>(topic_pub_range, 0);
   _pub_evo_tof_detailed = _nh.advertise<evo_rd_platform_example::evo_ToF_detailed>(
       topic_pub_detailed, 0);

   // finish flag
   evo::log::get() << _logger_prefix << "finished init process!" << evo::info;
   _is_initialized = true;
   return true;
}

void ToFControllerROS::publish_data(
    std::shared_ptr<evo_mbed::ToFSensor> actual_sensor, const std::string& frame_id,
    sensor_msgs::Range& range,
    evo_rd_platform_example::evo_ToF_detailed& tof_detailed)
{
   // evo detailed topic
   tof_detailed.header.stamp    = ros::Time::now();
   tof_detailed.header.frame_id = frame_id;
   tof_detailed.range           = actual_sensor->getDistanceMM() * 1000.0;
   tof_detailed.sigma           = actual_sensor->getSigmaMM();
   tof_detailed.status = static_cast<uint8_t>(actual_sensor->getRangeStatus());
   _pub_evo_tof_detailed.publish(tof_detailed);

   if(tof_detailed.status ==
      static_cast<uint8_t>(evo_mbed::ToFRangeStatus::TOF_RSTS_VLD))
   // if(tof_detailed.status == 0)
   {
      // classic range topic
      range.range           = actual_sensor->getDistanceMM() * 1000.0;
      range.header.frame_id = frame_id;
      range.header.stamp    = ros::Time::now();
      _pub_range.publish(range);
   }
}

void ToFControllerROS::main_loop()
{
   if(!_is_initialized)
   {
      evo::log::get() << _logger_prefix << "not initialized! --> return"
                      << evo::error;
      return;
   }
   else
   {
      evo::log::get() << _logger_prefix << "starting main control loop.."
                      << evo::info;

      // some constant values for all sensors
      sensor_msgs::Range range;
      range.min_range     = 0;
      range.field_of_view = _ToF_FoV_rad;
      range.max_range     = _ToF_range_max_m;

      evo_rd_platform_example::evo_ToF_detailed tof_detailed;
      tof_detailed.fov       = _ToF_FoV_rad;
      tof_detailed.max_range = _ToF_range_max_m;

      while(ros::ok())
      {
         ros::spinOnce();

         // go trough all sensors (frame id and sensor)
         for(auto& sensor_pair : _map_ToF_sensors)
         {
            // evo detailed topic
            tof_detailed.header.stamp    = ros::Time::now();
            tof_detailed.header.frame_id = sensor_pair.first;
            tof_detailed.range = sensor_pair.second->getDistanceMM() / 1000.0;
            tof_detailed.sigma = sensor_pair.second->getSigmaMM();
            tof_detailed.status =
                static_cast<uint8_t>(sensor_pair.second->getRangeStatus());
            _pub_evo_tof_detailed.publish(tof_detailed);

            if(tof_detailed.status ==
               static_cast<uint8_t>(evo_mbed::ToFRangeStatus::TOF_RSTS_VLD))
            // if(tof_detailed.status == 0)
            {
               // classic range topic
               range.range           = tof_detailed.range;
               range.header.frame_id = sensor_pair.first;
               range.header.stamp    = ros::Time::now();
               _pub_range.publish(range);
            }
         }
         _loop_rate_hz.sleep();
      }
   }
}

} // namespace evo
