#!/bin/bash

###############################################################
# Copyright (C) 2020, Evocortex GmbH, All rights reserved.    #
# Further regulations can be found in LICENSE file.           #
###############################################################

catkin clean evo_dcdc_shield_interface evo_logger evo_rd_platform_example evo_tof_board_interface evo_drives_control evo_motor_shield_interface evo_robot_base_interface 

catkin build evo_dcdc_shield_interface evo_logger evo_rd_platform_example evo_tof_board_interface evo_drives_control evo_motor_shield_interface evo_robot_base_interface 
