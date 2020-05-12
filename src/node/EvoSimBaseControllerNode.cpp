//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

#include "sw_interface/SimBaseControllerROS.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "EvoSimBaseControllerNode");

   evo::SimBaseControllerROS sbcr;

   sbcr.main_loop();
   exit(0);
}
