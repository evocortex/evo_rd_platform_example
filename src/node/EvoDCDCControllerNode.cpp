//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

#include "sw_interface/DCDCControllerROS.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "EvoDCDCControllerNode");

   evo::DCDCControllerROS dccr;

   dccr.init();
   dccr.main_loop();
   exit(0);
}
