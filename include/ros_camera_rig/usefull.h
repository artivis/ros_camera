/*
 *  usefull.h
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 07/2014
 *      Author: Jéremie Deray
 */

#ifndef ROS_CAMERA_RIG_USEFULL_H
#define ROS_CAMERA_RIG_USEFULL_H

#include <vector>
#include <set>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>


const double L_PI = 3.141592653589793238462643383279;

const float XI_DEFAULT = 0.;
const std::string CAMERA_TYPE_DEFAULT = "pinhole";

// xi > 1
const std::string CAMERA_TYPE_FISHEYE = "fisheye";

// 0 < xi < 1
const std::string CAMERA_TYPE_PARACATADIOPTRIC = "catadioptric";

const float XI_CATADIOPTRIC = 1.;
const std::string CAMERA_TYPE_CATADIOPTRIC = "paracatadioptric";

// Identity
const cv::Mat DEFAULT_EXTRINSIC_PARAM = cv::Mat::eye(4, 4, CV_32F);

// no disto
const cv::Mat DEFAULT_DISTORTION = cv::Mat::zeros(1, 5, CV_32F);

namespace cv
{
  class Chrono
  {
  public:
    Chrono() :
      _start(0.0) {}
    ~Chrono() {}

    // init timer
    double tic()
    {
      return _start = (double)cv::getTickCount();
    }

    // return timer time in seconds
    double toc(bool reset = false)
    {
      double stop = (double)cv::getTickCount();
      double diff = (stop - _start) / cv::getTickFrequency();

      if (reset)
        _start = stop;

      return diff;
    }

  private:

    double _start;
  };
}

namespace ros
{
  class Chrono
  {
  public:
    Chrono(bool nsec = false) :
      _start(0.0),
      _nsec(nsec) {}
    ~Chrono() {}

    // init timer
    double tic()
    {
      return _start = (_nsec) ? ros::Time::now().toNSec() :
                                ros::Time::now().toSec();
    }

    // return timer time in seconds
    double toc(bool reset = false)
    {
      double stop = (_nsec) ? ros::Time::now().toNSec() :
                              ros::Time::now().toSec();
      double diff = stop - _start;

      if (reset)
        _start = stop;

      return diff;
    }

  private:

    bool _nsec;
    double _start;
  };
}

template <class NumType>
cv::Mat Vector2Mat(std::vector< NumType > vect){

  cv::Mat matrix = cv::Mat::zeros(1,vect.size(),
                                  cv::DataType<NumType>::type);

  for (int r=0; r<vect.size(); r++)
    matrix.at<NumType>(0,r) = vect[r];

  return matrix;
}

bool cart2Sph(const cv::Mat& cart_coor,
              cv::Mat& sph_coor,
              int rad_flag = 0);

void sph2Cart(const cv::Mat &sph_pts, cv::Mat &cart_pts);

void sph2Heal(const cv::Mat &sph_pts, cv::Mat &heal_pts);

void meshGrid(const cv::Mat &X_val, const cv::Mat &Y_val,
              cv::Mat &X_grid, cv::Mat &Y_grid);

void rotateCloudPoint(cv::Mat &ClPts, float roll,
                      float pitch, float yaw,
                      bool rad = false);

cv::Mat getRotationMat(float roll, float pitch,
                       float yaw, bool rad = false);

cv::Mat getZYZRotationMat(double yaw1, double pitch,
                          double yaw2, bool rad = false);

double deg2Rad(double);

std::string addPath(const std::string& obj, const std::string& root);

void matInfo(const cv::Mat&,const std::string &matname = "",
             bool val = false);

void getListOfFilesInFolder(const std::string& path,
                            const std::string& extension,
                            std::vector<std::string>& baseFileNames);

std::set<std::string> loadFilesName(const std::string &dir);

std::string expand_tilde(std::string path);

#endif // ROS_CAMERA_RIG_USEFULL_H
