/*
 *  omni_camera.cpp
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 12/2014
 *      Author: Jéremie Deray
 */

#include "ros_camera_rig/camera_handler.h"

#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

CameraHandler::CameraHandler(const std::string &param_path) :
  _is_sampled(false),
  _sphLUT_init(false),
  _wrapLUT_init(false),
  _sampling_ratio(1.)
{
  _init = loadParam(param_path);
}

CameraHandler::CameraHandler(const CameraParam& cam_param) :
  _cam_param(cam_param),
  _is_sampled(false),
  _sphLUT_init(false),
  _wrapLUT_init(false),
  _sampling_ratio(1.)
{
  _init = true;
}

CameraHandler::~CameraHandler() {}

bool CameraHandler::loadParam(const std::string &paramPath)
{
  std::string param_path = expand_tilde(paramPath);

  cv::FileStorage fs(param_path.c_str(), cv::FileStorage::READ);

  if (!fs.isOpened())
  {
    ROS_ERROR_STREAM("Failed to open " << param_path);
    return false;
  }

  try
  {
    fs["type"] >> _cam_param.camera_type;
  }
  catch(cv::Exception &e)
  {
    ROS_ERROR("Error while reading camera type : %s", e.what());
    ROS_ERROR("Setting to default type : %s", CAMERA_TYPE_DEFAULT.c_str());
    _cam_param.camera_type = CAMERA_TYPE_DEFAULT;
  }

  try
  {
    fs["xi"] >> _cam_param.xi;
  }
  catch(cv::Exception &e)
  {
    ROS_ERROR("Error while reading xi : %s", e.what());
    ROS_ERROR("Setting to default xi : %f", XI_DEFAULT);
    _cam_param.xi = XI_DEFAULT;
  }

  try
  {
    fs["K"] >> _cam_param.K;
    _cam_param.K.convertTo(_cam_param.K, CV_32F);
  }
  catch(cv::Exception &e)
  {
    ROS_ERROR("Error while reading K : %s", e.what());
    ROS_ERROR("Please fix your config file");
    return false;
  }

  _cam_param.K_inv = _cam_param.K.inv();

  try
  {
    fs["pose"] >> _cam_param.cam_pose;
    _cam_param.cam_pose.convertTo(_cam_param.cam_pose, CV_32F);
  }
  catch(cv::Exception &e)
  {
    ROS_ERROR("Error while reading pose : %s", e.what());
    ROS_ERROR_STREAM("Setting to default pose :\n"
                     << DEFAULT_EXTRINSIC_PARAM);
    return false;
  }

  try
  {
    fs["distortion"] >> _cam_param.distortion;
    _cam_param.distortion.convertTo(_cam_param.distortion, CV_32F);
  }
  catch(cv::Exception &e)
  {
    ROS_ERROR("Error while reading distortion : %s", e.what());
    ROS_ERROR("Assume default distortion = no distortion");
    _cam_param.distortion = DEFAULT_DISTORTION;
  }

  try
  {
    cv::FileNode fn = fs["image_size"];
    fn["rows"] >> _cam_param.imsize.rows;
    fn["cols"] >> _cam_param.imsize.cols;
  }
  catch(cv::Exception &e)
  {
    ROS_ERROR("Error while reading image_size : %s", e.what());
    ROS_ERROR("Assume rows = K(0,2)*2");
    ROS_ERROR("Assume cols = K(1,2)*2");
    _cam_param.imsize.rows = (int)_cam_param.K.at<float>(0,2)*2;
    _cam_param.imsize.cols = (int)_cam_param.K.at<float>(1,2)*2;
  }

  fs.release();

  return true;
}

void CameraHandler::setImage(const cv::Mat &image)
{
  _frame = image.clone();

  if (_is_sampled)
    cv::resize(_frame, _frame, cv::Size(),
               1.0/_sampling_ratio,
               1.0/_sampling_ratio);
}

void CameraHandler::setImage(const cv::Mat &image,
                             boost::mutex& mut)
{
  boost::mutex::scoped_lock lock(mut);

  _frame = image.clone();

  if (_is_sampled)
    cv::resize(_frame, _frame, cv::Size(),
               1.0/_sampling_ratio,
               1.0/_sampling_ratio);
}

/*void Camera::ReadFrame(){

    sensor_msgs::ImageConstPtr frame;// = ImageHandler::waitUntilImageReceived();

    cv_bridge::CvImagePtr cvPtr;

    cvPtr = cv_bridge::toCvCopy(frame,"8UC3");

    _frame = cvPtr->image;

    if(_is_sampled) cv::resize(_frame,_frame,cv::Size(),1.0/_sampling_ratio,1.0/_sampling_ratio);
}*/


void CameraHandler::setIntrinsic(const cv::Mat &K)
{
  if (K.rows != 3 &&
      K.cols != 3)
    return;
  _cam_param.K = K;
  _cam_param.K_inv = K.inv();
}

void CameraHandler::setPose(const cv::Mat &pose)
{
  if (pose.rows != 4 &&
      pose.cols != 4)
    return;
  _cam_param.cam_pose = pose;
}


void CameraHandler::dispParam()
{
  ROS_INFO_STREAM("\ncamera type :\n" << _cam_param.camera_type <<
                  "\ncamera xi :\n" << _cam_param.xi <<
                  "\ncamera intrinsic parameters :\n " << _cam_param.K <<
                  "\nimage size :\n   rows : " << _cam_param.imsize.rows <<
                  "\n   cols : " << _cam_param.imsize.cols << "\n" <<
                  "camera pose :\n" << _cam_param.cam_pose);
}

void CameraHandler::loadMask(const std::string& mask_path)
{
  cv::Mat tmp;
  try
  {
    tmp = cv::imread(mask_path, CV_LOAD_IMAGE_GRAYSCALE);
  }
  catch(cv::Exception &e)
  {
    ROS_ERROR("Error while reading mask file : %s", e.what());
    ROS_ERROR("No mask loaded");
    return;
  }
  cv::threshold(tmp, tmp, 240, 1, cv::THRESH_BINARY);
  tmp.convertTo(_mask, CV_8UC1);
}

void CameraHandler::loadMask(const cv::Mat &mask)
{
  _mask = mask.clone();
}

/*void Camera::readImage(const std::string &file)
{
  ROS_DEBUG_STREAM("Reading image " << file);
  try
  {
    _frame = cv::imread(file);
  }
  catch(cv::Exception &e)
  {
    ROS_ERROR("Error while reading image %s : %s", file, e.what());
    return;
  }

  if(_is_sampled)
    cv::resize(_frame,_frame,cv::Size(),
               1.0/_sampling_ratio,1.0/_sampling_ratio);
}*/

cv::Vec3f CameraHandler::pix2Sph(int ind_row, int ind_col)
{
  cv::Vec3f pts;

  cv::Mat Kinv = getKinv();

  float x = (float)ind_col;
  float y = (float)ind_row;
  float z = 1.;

  x = Kinv.at<float>(0,0)*x + Kinv.at<float>(0,1)*y + Kinv.at<float>(0,2)*z;
  y = Kinv.at<float>(1,0)*x + Kinv.at<float>(1,1)*y + Kinv.at<float>(1,2)*z;
  z = Kinv.at<float>(2,0)*x + Kinv.at<float>(2,1)*y + Kinv.at<float>(2,2)*z;

  float alpha;

  alpha = (_cam_param.xi * z +
           sqrt(z*z + (1-_cam_param.xi*_cam_param.xi) * (x*x + y*y))
          ) / (x*x + y*y + z*z);

  pts[0] = x * alpha;
  pts[1] = y * alpha;
  pts[2] = z * alpha - _cam_param.xi;

  return pts;
}

void CameraHandler::im2Sph(int rows, int cols)
{
  if (!isInit()) return;

  if (_mask.empty())
    _mask = cv::Mat::ones(rows, cols, CV_8UC1);

  cv::Mat ind_nzero;
  cv::findNonZero(_mask, ind_nzero);
  int pixel_count = ind_nzero.rows * ind_nzero.cols;

  _LUT_sphere = cv::Mat::zeros(3, pixel_count, CV_32FC1);

  int i = 0;
  for (int row = 0; row < rows; ++row)
  {
    for (int col = 0; col < cols; ++col)
    {
      if (!(_mask.at<uchar>(row, col) == 0))
      {
        cv::Vec3f pts_sph = pix2Sph(row, col);

        _LUT_sphere.at<float>(0, i) = pts_sph[0];
        _LUT_sphere.at<float>(1, i) = pts_sph[1];
        _LUT_sphere.at<float>(2, i) = pts_sph[2];
      }
      ++i;
    }
  }

  // Apply pose rotation
  _LUT_sphere = _cam_param.cam_pose(cv::Rect(0,0,3,3)) * _LUT_sphere;

  _sphLUT_init = true;
}

void CameraHandler::im2Sph(const ImageSize &im)
{
  im2Sph(im.rows, im.cols);
}

void CameraHandler::im2Sph()
{
  im2Sph(_cam_param.imsize);
}

void CameraHandler::downSample(int sampling_ratio)
{
  if (!isInit()) return;

  _cam_param.imsize.cols /= sampling_ratio;
  _cam_param.imsize.rows /= sampling_ratio;

  _cam_param.K /= sampling_ratio;
  _cam_param.K.at<float>(2,2) = 1;
  _cam_param.K_inv = _cam_param.K.inv();

  if (!_frame.empty())
    cv::resize(_frame, _frame, cv::Size(),
               1.0/sampling_ratio, 1.0/sampling_ratio);

  if (!_mask.empty())
    cv::resize(_mask, _mask, cv::Size(),
               1.0/sampling_ratio, 1.0/sampling_ratio);

  if (!_LUT_sphere.empty())
    im2Sph(_cam_param.imsize.rows, _cam_param.imsize.cols);

  _is_sampled = true;
  _sampling_ratio = sampling_ratio;
}

/*
cv::Vec2i Camera::Sph2Im(float x, float y, float z)
{
  cv::Vec2i pix;

  pix[1] = (int)((x * _cam_param.K.at<float>(0,0)) /
            (z + _cam_param.xi) + _cam_param.K.at<float>(0,2));

  pix[0] = (int)((y * _cam_param.K.at<float>(1,1)) /
            (z + _cam_param.xi) + _cam_param.K.at<float>(1,2));

  return pix;
}
*/

bool CameraHandler::sph2Pano()
{
  if (!isInit())
    return false;

  if (_LUT_sphere.empty())
    return false;

  return _wrapLUT_init =
      cart2Sph(_LUT_sphere, _LUT_wrap_im);
}

