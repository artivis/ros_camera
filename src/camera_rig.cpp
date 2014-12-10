/*
 *  poly_omni.cpp
 *  Copyright (c) 2014 PAL Robotics sl. All Rights Reserved
 *  Created on: 12/2014
 *      Author: Jéremie Deray
 */

#include "ros_camera_rig/camera_rig.h"
#include "ros_camera_rig/point_cloud2_iterator.h"

#include <opencv2/highgui/highgui.hpp>


CameraRig::CameraRig() :
  _isSampled(false),
  _init(false),
  _sampling_ratio(1),
  _ind_LUTsph(0) {}

CameraRig::CameraRig(const std::vector<std::string> &cam_param_path,
                     cv::Size pano) :
  _isSampled(false),
  _init(false),
  _sampling_ratio(1),
  _ind_LUTsph(0)
{
  _init = addCameras(cam_param_path);

  _img_h.reset( new SyncImageRig() );

  ROS_DEBUG_STREAM("isInit : " << _init);

  _pano_size = pano;
}

bool CameraRig::addCameras(const std::vector<std::string> &cam_param_path)
{
  bool init = true;
  for (int i=0; i<cam_param_path.size(); ++i)
  {
    if (cam_param_path[i] != "none" && cam_param_path[i] != "")
      init = init && addCamera(cam_param_path[i]);
  }
  return init;
}

bool CameraRig::addCamera(const std::string &param_path)
{
  _cameras.push_back(new CameraHandler(param_path));
  return _cameras.back().isInit();
}

bool CameraRig::addCamera(const CameraHandler &cam_h)
{
  _cameras.push_back(new CameraHandler(cam_h));
  return _cameras.back().isInit();
}

void CameraRig::dispParam()
{
  for (int i=0; i<_cameras.size(); ++i)
  {
    ROS_INFO("Camera %i :", i);
    _cameras[i].dispParam();
  }
}

cv::Mat CameraRig::getExtrinsic(int i)
{
  if (i < _cameras.size())
    return _cameras[i].getPose();
  else
    ROS_ERROR("Index %i doesn't exist!", i);
  return cv::Mat::eye(4, 4, CV_64F);
}

cv::Mat CameraRig::getPano()
{
  return _pano;
}

cv::Mat CameraRig::getLUT()
{
  return _LUT_wrap_im;
}

void CameraRig::setExtrin(const cv::Mat &extrin, int index)
{
  if (index < _cameras.size())
    _cameras[index].setPose(extrin);
  else
    ROS_ERROR("Index %i doesn't exist!", index);
}

void CameraRig::setPanoSize(cv::Size panosize)
{
  _pano_size = panosize;
}

void CameraRig::setPanoSize(int rows, int cols)
{
  _pano_size = cv::Size(cols, rows);
}

bool CameraRig::assertLUTs(bool allut)
{
  if (!isInit())
    return false;

  bool assert;

  for (int i=0; i<_cameras.size(); ++i)
    assert = assert && _cameras[i].isInit()
              && _cameras[i].isSpheLUT();
  if (allut)
  {
    for (int i=0; i<_cameras.size(); ++i)
      assert = assert && _cameras[i].isWrapLUT();
  }

  return assert;
}

bool CameraRig::initLUTs(bool allut)
{
  if (isInit())
    for (int i=0; i<_cameras.size(); ++i)
    {
      if (allut)
      {
        _cameras[i].im2Sph();
        _cameras[i].sph2Pano();
      }
      else
        _cameras[i].im2Sph();
    }
}

void CameraRig::rescaleWrapLUT(cv::Size size)
{
    double min,max;

    cv::minMaxLoc(_LUT_wrap_im.row(0),&min,&max);

    _LUT_wrap_im.row(0).convertTo(_LUT_wrap_im.row(0), CV_16UC1,
                          (double)(size.width/(max-min)),
                          (double)(- (min * (size.width/(max-min)))));

    cv::minMaxLoc(_LUT_wrap_im.row(1),&min,&max);

    _LUT_wrap_im.row(1).convertTo(_LUT_wrap_im.row(1),CV_16UC1,
                         size.height/(max-min),
                         - (min * (size.height/(max-min))));

    _pano = cv::Mat::zeros(size.height,size.width,_pano.type());
}

bool CameraRig::updateImage()
{
  _images.clear();
  bool update =_img_h->waitForImages(_images, ros::Duration(0, 5e-5));

  if (_sampling_ratio != 1 &&
      _sampling_ratio != 0)
  {
    for (int i=0; i<_images.size(); ++i)
    {
      cv::resize(_images[i], _images[i], cv::Size(),
                 1.0/_sampling_ratio, 1.0/_sampling_ratio);
    }
  }
  return update;
}

// TODO
/*
bool CameraRig::stitchImage(bool INPAIN_FLAG)
{
  if (!isInit() && !assertLUTs())
    return false;

  // If no mask, use all pixels
  for (int i=0; i<_cameras.size(); ++i)
    if (_cameras[i]->_mask.empty())
      _cameras[i]->_Mask = cv::Mat::ones(_cameras[i]->_cam_param.imsize.rows,
                                         _cameras[i]->_cam_param.imSize.cols,
                                         CV_8UC1);

  _pano = cv::Mat::zeros(_pano_size, CV_8UC3);

  cv::Mat mask_inpaint = cv::Mat::zeros(_pano_size, 0);
  mask_inpaint += 255;

  int pix_im1 = camera_1->_cameraParam.imSize.cols * camera_1->_cameraParam.imSize.rows;

  cv::Mat im_val = camera_1->_Frame;

  cv::Mat im_mask = camera_1->_Mask;

  int _cross_ind_row = 0;
  int _cross_ind_col = 0;

  bool update = updateImage();

  if (!update)
    return false;

  for (int i=0; i<_images.size(); ++i)
  {
    int chan = _images[i].channels();
    int nRows = _images[i].rows;
    int nCols = _images[i].cols * chan;

    if (_images[i].isContinuous() &&
        _cameras[i]->_mask.isContinuous())
    {
      nCols *= nRows;
      nRows = 1;
    }

    int i, j;
    int indLUTrow = indLUTcol = 0;
    uchar* img_ptr, mask_ptr;

    for (i = 0; i < nRows; ++i)
    {
      img_ptr = _images[i].ptr<uchar>(i);
      mask_ptr = _cameras[i]->_mask.ptr<uchar>(i);

      for (j = 0; j < nCols; ++j)
      {
        p[j] = table[p[j]];

        if (mask_ptr[j] > 0)
        {

          _pano.at<cv::Vec3b>(_LUT_wrap_im.at<unsigned short>(0,i),_LUT_wrap_im.at<unsigned short>(1,i)) =
                    im_val.at<cv::Vec3b>(_cross_ind_row,_cross_ind_col);


            if (INPAIN_FLAG) mask_inpaint.at<uchar>(_LUT_wrap_im.at<unsigned short>(0,i),_LUT_wrap_im.at<unsigned short>(1,i)) = 0;
        }
        _cross_ind_row++;

        if (_cross_ind_row == im_val.rows)
        {
            _cross_ind_row = 0;
            _cross_ind_col++;
        }

        if (i == pix_im1-1)
        {
            im_val = camera_2->_Frame;
            im_mask = camera_2->_Mask;
            _cross_ind_row = 0;
            _cross_ind_col = 0;
        }

      }
    }
  }

//////////////////////////////////////////////////////////////

    for (int i = 0; i < _LUT_wrap_im.cols; i++)
    {
        if(im_mask.at<uchar>(_cross_ind_row,_cross_ind_col) > 0)
        {

            _pano.at<cv::Vec3b>(_LUT_wrap_im.at<unsigned short>(0,i),
                                _LUT_wrap_im.at<unsigned short>(1,i)) =
                    im_val.at<cv::Vec3b>(_cross_ind_row,_cross_ind_col);


            if (INPAIN_FLAG)
              mask_inpaint.at<uchar>(_LUT_wrap_im.at<unsigned short>(0,i),
                                     _LUT_wrap_im.at<unsigned short>(1,i)) = 0;
        }
        _cross_ind_row++;

        if (_cross_ind_row == im_val.rows)
        {
            _cross_ind_row = 0;
            _cross_ind_col++;
        }

        if (i == pix_im1-1)
        {
            im_val = camera_2->_Frame;
            im_mask = camera_2->_Mask;
            _cross_ind_row = 0;
            _cross_ind_col = 0;
        }
    }

    cv::inpaint(_pano,mask_inpaint,_pano,5,cv::INPAINT_TELEA);
}
*/

bool CameraRig::saveImage(const std::string &filename)
{
  try
  {
    cv::imwrite(filename, _pano);
  }
  catch(cv::Exception &e)
  {
    ROS_ERROR("Error while saving image : %s", e.what());
    return false;
  }
  return true;
}

void CameraRig::setCamImage(const cv::Mat &image, int i)
{
  if(!isInit())
    return;

  _cameras[i].setImage(image.clone());
}

bool CameraRig::partiallyFillMess(sensor_msgs::PointCloud2 &PointCloud)
{
  if (!isInit())
    return false;

  // assert that at least sph pts LUT
  // is init &
  // retrieve total number of sph pts
  int mess_size = 0;
  if (!assertLUTs(false))
  {
    for (int i=0; i<_cameras.size(); ++i)
    {
      _cameras[i].im2Sph();
      mess_size += _cameras[i].getSphLUT().cols;
    }
  }

  PointCloud.header.frame_id = "map";

  PointCloud.height = 1;
  PointCloud.width = mess_size;

  sensor_msgs::PointCloud2Modifier modifier(PointCloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(PointCloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(PointCloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(PointCloud, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(PointCloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(PointCloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(PointCloud, "b");

  for (int i=0; i<_cameras.size(); ++i)
  {
    cv::Mat LUT_ptr = _cameras[i].getSphLUT();

    for (int j=0; j<LUT_ptr.cols; ++j)
    {
      *iter_x = LUT_ptr.at<float>(0, j);
      *iter_y = LUT_ptr.at<float>(1, j);
      *iter_z = LUT_ptr.at<float>(2, j);

      *iter_r = 0;
      *iter_g = 0;
      *iter_b = 0;

      ++iter_x;
      ++iter_y;
      ++iter_z;

      ++iter_r;
      ++iter_g;
      ++iter_b;
    }
  }

  ROS_INFO_STREAM("PointCloud msg size : " << PointCloud.width << " \n");

  return true;
}

bool CameraRig::partiallyFillMess(Ogre::ManualObject *obj)
{
  if (!isInit())
    return false;

  // assert that at least sph pts LUT
  // is init &
  // retrieve total number of sph pts
  int mess_size = 0;
  if (!assertLUTs(false))
  {
    for (int i=0; i<_cameras.size(); ++i)
    {
      _cameras[i].im2Sph();
      mess_size += _cameras[i].getSphLUT().cols;
    }
  }

  // Always tell if you want to update the 3D (vertex/index) later or not.
  obj->setDynamic(true);

  obj->estimateVertexCount(mess_size);

  obj->begin("M_Lighting", Ogre::RenderOperation::OT_POINT_LIST, "General");
  {
    for (int i=0; i<_cameras.size(); ++i)
    {
      cv::Mat LUT_ptr = _cameras[i].getSphLUT();

      for (int j=0; j<LUT_ptr.cols; ++j)
      {
        // a vertex
        obj->position(LUT_ptr.at<float>(0, j),  //x
                      LUT_ptr.at<float>(1, j),  //y
                      LUT_ptr.at<float>(2, j)); //z
        obj->colour(Ogre::ColourValue(1.0f, 1.0f, 1.0f, 1.0f)); // white color

      }
    }
  }
  obj->end();

  ROS_INFO_STREAM("PointCloud size : " << mess_size << " \n");

  return true;
}

bool CameraRig::messRGBSph(Ogre::ManualObject *obj)
{
  // WTF ! without this ros_debug, if(!isInit())
  // fails !!!
  ROS_DEBUG("!isInit()");
  if (!isInit())
    return false;

  if (!assertLUTs(false))
    return false;

  if (!updateImage())
    return false;

  obj->beginUpdate(0);
  {
    for (int c = 0; c < _cameras.size(); ++c)
    {
      int channels = _images[c].channels();

      int nRows = _images[c].rows;
      int nCols = _images[c].cols * channels;

      if (_images[c].isContinuous() &&
          _cameras[c].getMask().isContinuous())
      {
        nCols *= nRows;
        nRows = 1;
      }

      int ind_mask = 0;
      uchar *img_ptr, *mask_ptr;

      cv::Mat LUT_ptr = _cameras[c].getSphLUT();
      int k = 0;

      for (int i = 0; i < nRows; ++i)
      {
        img_ptr = _images[c].ptr<uchar>(i);
        mask_ptr = _cameras[c].getMask().ptr<uchar>(i);

        for (int j = 0; j < nCols; j+=3)
        {
          if (mask_ptr[ind_mask] > 0.)
          {
            // a vertex
            obj->position(LUT_ptr.at<float>(0, k),  //x
                          LUT_ptr.at<float>(1, k),  //y
                          LUT_ptr.at<float>(2, k)); //z

            obj->colour(Ogre::ColourValue(((float)(img_ptr[j]))/255.f,
                                          ((float)(img_ptr[j+1]))/255.f,
                                          ((float)(img_ptr[j+2]))/255.f,
                                          1.0f)); // color

            ++k;
          }
          ++ind_mask;
        }
      }
    }
  }
  obj->end();

  return true;
}

bool CameraRig::messRGBSph(sensor_msgs::PointCloud2 &PointCloud)
{
  // WTF ! without this ros_debug, if(!isInit())
  // fails !!!
  ROS_DEBUG("!isInit()");
  if (!isInit())
    return false;

  if (!assertLUTs(false))
    return false;

  if (PointCloud.width == 0)
    return false;

  if (!updateImage())
    return false;

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(PointCloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(PointCloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(PointCloud, "b");

  for (int c = 0; c < _cameras.size(); ++c)
  {
    int channels = _images[c].channels();

    int nRows = _images[c].rows;
    int nCols = _images[c].cols * channels;

    if (_images[c].isContinuous() &&
        _cameras[c].getMask().isContinuous())
    {
      nCols *= nRows;
      nRows = 1;
    }

    int ind_mask = 0;
    uchar *img_ptr, *mask_ptr;
    for (int i = 0; i < nRows; ++i)
    {
      img_ptr = _images[c].ptr<uchar>(i);
      mask_ptr = _cameras[c].getMask().ptr<uchar>(i);

      for (int j = 0; j < nCols; j+=3)
      {
        if (mask_ptr[ind_mask] > 0.)
        {
          *iter_r = (float)((img_ptr[j])); //r
          *iter_g = (float)((img_ptr[j+1])); //g
          *iter_b = (float)((img_ptr[j+2])); //b

          ++iter_b;
          ++iter_g;
          ++iter_r;
        }
        ++ind_mask;
      }
    }
  }

  PointCloud.header.stamp = ros::Time::now();
  return true;
}

void CameraRig::downSample(int sampling_ratio)
{
  if (!isInit() || sampling_ratio == 1)
    return;

  for (int i=0; i<_cameras.size(); ++i)
  {
    _cameras[i].downSample(sampling_ratio);
  }

    _isSampled = true;
    _sampling_ratio = sampling_ratio;
}

/*
void CameraRig::sph2Pano()
{
    if (!isInit()) return;
    if (_LUTsphere.empty()) return;

    double min,max;

    cv::Mat tmp;

    rotate90roll();

    Cart2Sph(_LUTsphere, tmp);

    _LUT_wrap_im = cv::Mat::zeros(tmp.size(),CV_16UC1);

    cv::minMaxLoc(tmp.row(0),&min,&max);

    tmp.row(0).convertTo(_LUT_wrap_im.row(0), CV_16UC1
                         ,(double)((_pano_size.height-1)/(max-min)),(double)(- (min * ((_pano_size.height-1)/(max-min)))));

    cv::minMaxLoc(tmp.row(1),&min,&max);

    tmp.row(1).convertTo(_LUT_wrap_im.row(1), CV_16UC1
                         ,(double)((_pano_size.width-1)/(max-min)),(double)(- (min * ((_pano_size.width-1)/(max-min)))));
}

void CameraRig::sph2HealPano()
{
    if (!isInit()) return;
    if (_LUTsphere.empty()) return;

    double min,max;

    cv::Mat tmp, tmp2;

    rotate90roll();

    Cart2Sph(_LUTsphere, tmp);

    Sph2Heal(tmp,tmp2);

    tmp.release();

    _LUT_wrap_im = cv::Mat::zeros(tmp2.size(),CV_16UC1);

    cv::minMaxLoc(tmp2.row(0),&min,&max);

    tmp2.row(0).convertTo(_LUT_wrap_im.row(0), CV_16UC1
                          ,(double)((_pano_size.height-1)/(max-min)),(double)(- (min * ((_pano_size.height-1)/(max-min)))));

    cv::minMaxLoc(tmp2.row(1),&min,&max);

    tmp2.row(1).convertTo(_LUT_wrap_im.row(1), CV_16UC1
                          ,(double)((_pano_size.width-1)/(max-min)),(double)(- (min * ((_pano_size.width-1)/(max-min)))));

}

*/
