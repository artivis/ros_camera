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

  /**
  * getImages()
  * Gets updated images from SyncImageDisplay object
  * @param images : std::vector< cv::Mat >
  *                 this vector is first cleared then
  *                 filled with images
  * @param timeout : max duration to wait for images
  */
  bool waitForImages(std::vector<cv::Mat>& images,
                     ros::Duration timeout = ros::Duration(0));

  /**
  * getTopics()
  * Returns topics that are listened
  */
  std::vector<std::string> getTopics()
  { return _topics; }

protected:

  /**
  * A defined virtual member.
  * @param vecImgPtr : std::vector< sensor_msgs::ImageConstPtr >
  */
  virtual void callback(const std::vector<MPtr>& vecImgPtr);

  boost::mutex img_mut;
  std::vector<cv::Mat> _images;
  bool _new_img;
};

#endif  // ROS_CAMERA_RIG_IMAGE_HANDLER_H_
