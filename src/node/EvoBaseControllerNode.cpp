//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

#include "sw_interface/BaseControllerROS.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "EvoBaseControllerNode");

   evo::BaseControllerROS bcr;

   bcr.init();
   bcr.main_loop();
   exit(0);
}
