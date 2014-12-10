/*
 *  omni_camera.h
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 12/2014
 *      Author: Jéremie Deray
 */

#ifndef ROS_CAMERA_RIG_CAMERA_HANDLER_H_
#define ROS_CAMERA_RIG_CAMERA_HANDLER_H_

#include "ros_camera_rig/usefull.h"

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <complex>

// OpenCV header
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ROS header
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

// BOOST header
//#include <boost/noncopyable.hpp>
#include <boost/utility.hpp>
#include <boost/thread/mutex.hpp>

struct ImageSize
{
  int rows;
  int cols;

  ImageSize() : rows(0), cols(0) {}

  ImageSize(int rows_, int cols_) :
    rows(rows_), cols(cols_) {}

  ImageSize(const ImageSize &imsize) :
    rows(imsize.rows), cols(imsize.cols) {}
};

struct CameraParam
{
  double xi;
  cv::Mat K;
  cv::Mat K_inv;
  cv::Mat distortion;
  ImageSize imsize;
  std::string camera_type;

  cv::Mat cam_pose;

  CameraParam() : camera_type(""), xi(0),
                  K(cv::Mat::zeros(3, 3, CV_32F)),
                  K_inv(K.inv()),
                  distortion(cv::Mat::zeros(1, 5, CV_32F)),
                  imsize(0, 0),
                  cam_pose(cv::Mat::eye(4, 4, CV_32F)){}

  CameraParam(const std::string& camT_, double xi_,
              const cv::Mat& param_,
              const ImageSize& imSize_,
              const cv::Mat& distortion_) :
              camera_type(camT_), xi(xi_),
              K(param_), K_inv(param_.inv()),
              imsize(imSize_),
              distortion(distortion_) {}

  CameraParam(const CameraParam &cam_param) :
              camera_type(cam_param.camera_type),
              xi(cam_param.xi), K(cam_param.K),
              K_inv(K.inv()), imsize(cam_param.imsize),
              distortion(cam_param.distortion),
              cam_pose(cam_param.cam_pose) {}
};


class CameraHandler //: boost::noncopyable
{

public:

  CameraHandler();
  CameraHandler(const CameraParam &cam_param);
  CameraHandler(const std::string &param_path);

  ~CameraHandler();

  bool loadParam(const std::string &paramPath);

  void dispParam();

  void loadMask(const std::string &mask_path);
  void loadMask(const cv::Mat &mask);

  // pixel index to cart coord onto unit sphere
  cv::Vec3f pix2Sph(int ind_row, int ind_col);

  // pixels index to cart coord onto unit sphere
  // stored in _LUTsphere
  void im2Sph(const ImageSize &imsize);
  void im2Sph(int rows, int cols);
  void im2Sph();

  // cart coord onto unit sphere
  // to spherical coord
  // stored in _LUT_wrap_im
  bool sph2Pano();

  //void Sph2Im(const cv::Mat&);
  //void Sph2Im(const cv::Mat&, cv::Mat&);
  //cv::Vec2i Sph2Im(float, float, float);

  void downSample(int sampling_ratio = 1);

  ////////////////////////////////////////////
  ////////// Get & Set functions /////////////
  ////////////////////////////////////////////

  double getXi()
    { return _cam_param.xi; }
  cv::Mat getMask()
    { return _mask; }
  cv::Mat getImage()
    { return _frame; }
  std::string getType()
    { return _cam_param.camera_type; }
  cv::Mat getIntrinsic()
    { return _cam_param.K; }
  cv::Mat getKinv()
    { return _cam_param.K_inv; }
  cv::Mat getPose()
    { return _cam_param.cam_pose; }
  ImageSize getImageSize()
    { return _cam_param.imsize; }
  int getPixelCount()
  {
    return _cam_param.imsize.cols *
           _cam_param.imsize.rows;
  }

  cv::Mat getSphLUT()
  { return _LUT_sphere; }

  //cv::Mat GetLUT() {return _LUTsphere;}
  //cv::Mat GetLUT(const std::string &);

  bool isSampled()
    { return _is_sampled; }

  bool isInit()
    {return _init;}

  bool isSpheLUT()
    {return _sphLUT_init;}

  bool isWrapLUT()
    {return _wrapLUT_init;}

  void setXi(double xi)
    {_cam_param.xi = xi;}
  void setType(const std::string &type)
    {_cam_param.camera_type = type;}
  void setImageSize(const ImageSize &imsize)
    {_cam_param.imsize = imsize;}
  void setImageSize(int rows, int cols)
    {setImageSize(ImageSize(rows, cols));}

  void setImage(const cv::Mat &img,
                boost::mutex& mut);
  void setImage(const cv::Mat &img);

  void setIntrinsic(const cv::Mat &K);
  void setPose(const cv::Mat &pose);

private :

  CameraParam _cam_param;

  cv::Mat _LUT_sphere;
  cv::Mat _LUT_wrap_im;

  cv::Mat _mask;
  cv::Mat _frame;

  bool _init;
  bool _sphLUT_init;
  bool _wrapLUT_init;
  bool _is_sampled;
  float _sampling_ratio;
};

#endif // ROS_CAMERA_RIG_CAMERA_HANDLER_H_
