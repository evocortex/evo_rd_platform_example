//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file ToFControllerROS.h
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

#ifndef TOFCONTROLLERROS_H
#define TOFCONTROLLERROS_H

// ROS header
#include "ros/ros.h"
#include "sensor_msgs/Range.h"

// custom msg
#include "evo_rd_platform_example/evo_ToF_detailed.h"

// ToF HW Header
#include "evo_tof_interface/ToFBoard.h"
#include "evo_tof_interface/ToFSensor.h"
#include "evo_mbed/tools/com/ComServer.h"

// Logger
#include "evo_logger/log/Logger.h"

namespace evo {

class ToFControllerROS
{
 private:
   std::string _logger_prefix;

   std::shared_ptr<evo_mbed::ComServer> _can_interface;
   std::map<std::string, std::shared_ptr<evo_mbed::ToFBoard>> _map_ToF_boards;
   std::map<std::string, std::shared_ptr<evo_mbed::ToFSensor>> _map_ToF_sensors;

   double _ToF_FoV_rad;
   double _ToF_range_max_m;

   ros::NodeHandle _nh;
   ros::Publisher _pub_evo_tof_detailed;
   ros::Publisher _pub_range;

   ros::Rate _loop_rate_hz;

   bool _is_initialized;

 public:
   ToFControllerROS();

   bool init();

   void main_loop();

   void publish_data(std::shared_ptr<evo_mbed::ToFSensor> actual_sensor,
                     const std::string& frame_id, sensor_msgs::Range& range,
                     evo_rd_platform_example::evo_ToF_detailed& tof_detailed);
};

} // namespace evo
#endif // TOFCONTROLLERROS_H
