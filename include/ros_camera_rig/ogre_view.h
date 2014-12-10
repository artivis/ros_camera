#ifndef ROS_CAMERA_RIG_OGRE_VIEW_H_
#define ROS_CAMERA_RIG_OGRE_VIEW_H_

#include <iostream>
#include <boost/thread/thread.hpp>

#include <OGRE/Ogre.h>
#include <OGRE/OgreHardwareVertexBuffer.h>
#include <OGRE/OgreHardwareBufferManager.h>
#include <OGRE/OgreHardwareBuffer.h>
#include "ros_camera_rig/ogre_oculus.h"
#include <sensor_msgs/PointCloud2.h>
#include "ros_camera_rig/point_cloud2_iterator.h"

namespace
{
  typedef Ogre::String String;
  typedef std::vector<String> Strings;
}

class ViewerOgre
{
public:

  ViewerOgre(String path, bool win = false);

  ~ViewerOgre();

  Ogre::ManualObject* createManualObj();

  Ogre::ManualObject* getManualObj();

  bool loadUnicMesh(sensor_msgs::PointCloud2& obj);

  bool loadUnicMesh(Ogre::ManualObject *obj);

  bool update(sensor_msgs::PointCloud2& obj);

  bool update();

  bool isOgreInit();

  bool isInit();

private:

  bool _initOgre(Ogre::String, bool win = false);

  void _updateMesh(sensor_msgs::PointCloud2& obj);

  bool _isOgreInit;

  void _messagePump();

  Ogre::String _meshName;

  boost::shared_ptr<Ogre::Root>                 _rootPtr;
  boost::shared_ptr<Ogre::SceneManager>         _sceneMgrPtr;
  boost::shared_ptr<Ogre::RenderWindow>         _windowPtr;

  boost::shared_ptr<oculus_rviz_plugins::Oculus> _oculus;
};

#endif // ROS_CAMERA_RIG_OGRE_VIEW_H_
