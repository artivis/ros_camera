/*
 *  image_handler.h
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 12/2014
 *      Author: Jéremie Deray
 */


#ifndef ROS_CAMERA_RIG_IMAGE_HANDLER_H_
#define ROS_CAMERA_RIG_IMAGE_HANDLER_H_

// SyncImageHandler header
#include <ros_img_sync/sync_image_transport_handler.h>

// ROS headers
#include <ros/ros.h>

// BOOST headers
#include <boost/thread/mutex.hpp>

/**
* class SyncImageDisplay
* It synchronises image topic callbacks (up to 8)
* Its callback store images in a member
*/
class SyncImageRig :
  public SyncImageTransportHandler
{

public:

  /**
  * Constructor.
  * @see SyncImageHandler
  */
  SyncImageRig();

  ~SyncImageRig() {}

protected:

};

#endif  // ROS_CAMERA_RIG_IMAGE_HANDLER_H_
