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
  SyncImageTransportHandler(), // don't forget to call base constructor first
  _new_img(false)
{
  for (int i=0; i<_topics.size(); ++i)
    _images.push_back(cv::Mat());
}

/**
* getImages()
* Gets updated images from SyncImageDisplay object
* @param images : std::vector< cv::Mat >
*                 this vector is first cleared then
*                 filled with images
* @param timeout : max duration to wait for images
*/
bool SyncImageRig::waitForImages(std::vector<cv::Mat>& images,
                                 ros::Duration timeout)
{
  ros::Duration sleep(0, 10e-9/30); // wait for 1/30 sec
  ros::Time start = ros::Time::now();

  _new_img = false;

  while (!_new_img)
  {
    //ROS_ERROR_STREAM("wait1...");
    if (timeout != ros::Duration(0))
    {
      //ROS_ERROR_STREAM("wait2...");
      if ( (ros::Time::now() - start).toSec() > timeout.toSec() )
      {
        //ROS_ERROR_STREAM("wait3...");
        return false;
      }
    }

    sleep.sleep();
  }

  boost::mutex::scoped_lock lock(img_mut);

  for (int i=0; i<_images.size(); ++i)
    images.push_back(_images[i].clone());

  return true;
}

/**
* A defined virtual member.
* @param vecImgPtr : std::vector< sensor_msgs::ImageConstPtr >
*/
void SyncImageRig::callback(const std::vector<MPtr>& vecImgPtr)
{
  boost::mutex::scoped_lock lock(img_mut);

  for (int i=0; i<vecImgPtr.size(); ++i)
  {
    if (vecImgPtr[i].use_count() > 0)
    {
      try
      {
        // Here we clone OpenCV Mat to avoid messing up with pointers
        _images[i] = cv_bridge::toCvShare(vecImgPtr[i],
                                          vecImgPtr[i]->encoding)->image.clone();
      }
      catch (cv_bridge::Exception &e)
      {
        ROS_ERROR("Couldn't convert %s image", vecImgPtr[i]->encoding.c_str());
        _images[i] = cv::Mat();
      }
    }
  }

  _new_img = true;
}
