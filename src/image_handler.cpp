/*
 *  image_handler.cpp
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 12/2014
 *      Author: Jéremie Deray
 */

#include "ros_camera_rig/image_handler.h"

// OpenCV header
#include <opencv2/highgui/highgui.hpp>

// ROS header
#include <cv_bridge/cv_bridge.h>

/**
* Constructor.
* @see SyncImageHandler
*/
SyncImageRig::SyncImageRig() :
  SyncImageTransportHandler() // don't forget to call base constructor first
{
  if (!start())
    _nh.shutdown();
}

