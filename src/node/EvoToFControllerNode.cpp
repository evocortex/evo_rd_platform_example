//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

#include "sw_interface/ToFControllerROS.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "EvoToFControllerNode");

   evo::ToFControllerROS tcr;

   tcr.init();
   tcr.main_loop();
   exit(0);
}
