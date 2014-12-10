/*
 *  poly_omni.h
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 12/2014
 *      Author: Jéremie Deray
 */

#ifndef ROS_CAMERA_RIG_CAMERA_RIG_H_
#define ROS_CAMERA_RIG_CAMERA_RIG_H_

#include "ros_camera_rig/image_handler.h"
#include "ros_camera_rig/camera_handler.h"

#include <stdio.h>

// OpenCV header
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>

// ROS header
#include <sensor_msgs/PointCloud2.h>

// BOOST header
#include <boost/utility.hpp>
#include <boost/scoped_ptr.hpp>

// OGRE
#include <OGRE/OgreManualObject.h>

class CameraRig : boost::noncopyable
{// Handle cameras rig

public :

  CameraRig();
  CameraRig(const std::vector<std::string> &cam_param_path,
            cv::Size pano = cv::Size(1200,400));

  ~CameraRig() {}

  // Add camera to the camera rig
  bool addCamera(const std::string &param_path);
  bool addCamera(const CameraHandler &cam_h);
  bool addCameras(const std::vector<std::string> &cam_param_path);

  void dispParam();

  bool initLUTs(bool allut = true);

  //void mergeLUTSph(bool heal = false);

  bool updateImage();

  void rescaleWrapLUT(cv::Size size = cv::Size(1200,400));

  // Create Rig Panoramic Image from Cameras Image
  bool stitchImage(bool INPAIN_FLAG = 0);

  bool saveImage(const std::string &filename = "panoramicImage.jpg");

  // Pre-compute 3D Points according to Spherical Rig Image
  bool partiallyFillMess(sensor_msgs::PointCloud2 &);
  bool partiallyFillMess(Ogre::ManualObject *);

  // Fill Message with Colors from Spherical Rig Image
  bool messRGBSph(sensor_msgs::PointCloud2 &);
  bool messRGBSph(Ogre::ManualObject *);

  void setCamImage(const cv::Mat &image, int i);

  // Down Sample Cameras ~> K/samp_ratio
  // Images will be Down Sampled too
  void downSample(int sampling_ratio = 1);

  // make pano image
  void sph2Pano();

  // make pano image
  // healpix style
  void sph2HealPano();

  ////////////////////////////////////////////
  ////////// Get & Set functions /////////////
  ////////////////////////////////////////////

  cv::Mat getExtrinsic(int i);
  cv::Mat getPano();
  cv::Mat getLUT();

  void setExtrin(const cv::Mat &, int index);
  void setPanoSize(cv::Size panosize);
  void setPanoSize(int rows, int cols);

  bool isInit() {return _init;}

private :

  boost::scoped_ptr<SyncImageRig> _img_h;
  boost::ptr_vector<CameraHandler> _cameras;

  bool assertLUTs(bool allut = true);

  std::vector<cv::Mat> _images;
  cv::Size _pano_size;
  cv::Mat _pano;

  //Projection of Camera Rig Pixels
  //onto Unit Sphere
  cv::Mat _LUTsphere;

  //Projection of Camera Rig Spherical Pixels
  //onto Image Plan - Panoramic Image
  cv::Mat _LUT_wrap_im;

  bool _init;
  bool _isSampled;

  int _sampling_ratio;
  int _ind_LUTsph;
};

#endif // ROS_CAMERA_RIG_CAMERA_RIG_H_

