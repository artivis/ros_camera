/*
 *  demo_sperical_oculus.cpp
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 12/2014
 *      Author: Jéremie Deray
 */

#include "ros_camera_rig/image_handler.h"
#include "ros_camera_rig/camera_rig.h"

#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc,argv, "demo_pointcloud");

  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");

  double sampling_ratio;
  nh_param.param("sampling_ratio", sampling_ratio, 4.0);

  std::vector<std::string> cams_param;
  nh_param.getParam("cams_param", cams_param);

  if (cams_param.size() == 0)
  {
    ROS_ERROR_STREAM("cams_param vec size : " << cams_param.size());
    return -1;
  }

  CameraRig cam_rig(cams_param, cv::Size(1200,400));

  if (!cam_rig.isInit())
  {
    ROS_ERROR("Something went wrong, shutting down.");
    return -1;
  }

  cam_rig.dispParam();

  if (sampling_ratio > 1)
  {
    cam_rig.downSample(sampling_ratio);
    cam_rig.dispParam();
  }

  ros::Publisher pub_CloudSph = nh.advertise<sensor_msgs::PointCloud2>("cloud_sphere", 1);
  sensor_msgs::PointCloud2 ptsCld;

  ros::Chrono timer;

  cam_rig.partiallyFillMess(ptsCld);

  // AsyncSpinner : we have a thread for
  // the synchronized listeners & one
  // for process
  ros::AsyncSpinner aspin(1);
  aspin.start();

  do
  {
    timer.tic();

    if (!cam_rig.messRGBSph(ptsCld))
      continue;

    ROS_DEBUG_STREAM("PointCloud msg filled in " << timer.toc(1) << " seconds"); //ROS_INFO_STREAM

    pub_CloudSph.publish(ptsCld);

    ROS_DEBUG_STREAM("PointCloud displayed in " << timer.toc(1) << " seconds"); //ROS_INFO_STREAM

  } while (ros::ok());

  aspin.stop();

  return 1;
}

