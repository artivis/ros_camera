
#include "ros_camera_rig/ogre_view.h"

#include <ros/ros.h>

ViewerOgre::ViewerOgre(Ogre::String path, bool win) :
  _isOgreInit(false),
  _meshName("mySphere")
{
  _oculus.reset( new oculus_rviz_plugins::Oculus() );
  _isOgreInit = _initOgre(path, win);

  _windowPtr->setActive(true);
  _windowPtr->setAutoUpdated(false);

  _rootPtr->clearEventTimes();

  ROS_INFO("Ogre viewer's initialised !");
}

ViewerOgre::~ViewerOgre()
{
  _windowPtr->removeAllViewports();
  _sceneMgrPtr->destroyAllCameras();
  _sceneMgrPtr->destroyAllManualObjects();
  _sceneMgrPtr->destroyAllEntities();
  _sceneMgrPtr->destroyAllLights();
  _sceneMgrPtr->getRootSceneNode()->removeAndDestroyAllChildren();
  Ogre::ResourceGroupManager::getSingleton().
      destroyResourceGroup("General");
  _oculus->shutDownOculus();
  _oculus->shutDownOgre();
}


Ogre::ManualObject* ViewerOgre::createManualObj()
{
  return _sceneMgrPtr->createManualObject(_meshName);
}

Ogre::ManualObject* ViewerOgre::getManualObj()
{
  return _sceneMgrPtr->getManualObject(_meshName);
}

bool ViewerOgre::loadUnicMesh(sensor_msgs::PointCloud2& obj)
{
  if (! _isOgreInit)
  {
    ROS_ERROR("Viewer not init !");
    return false;
  }

  ROS_INFO("Loading Mesh");

  Ogre::ManualObject * lManualObject = NULL;
  {
    lManualObject = _sceneMgrPtr->createManualObject(_meshName);
    lManualObject->estimateVertexCount(obj.width);

    // Always tell if you want to update the 3D (vertex/index) later or not.
    lManualObject->setDynamic(true);

    lManualObject->begin("M_Lighting", Ogre::RenderOperation::OT_POINT_LIST, "General");
    {
      sensor_msgs::PointCloud2Iterator<float> iter_x(obj, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(obj, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(obj, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      {
        // populate with some data
        lManualObject->position(*iter_x, *iter_y, *iter_z);// a vertex
        lManualObject->colour(Ogre::ColourValue(1.0f, 0.0f, 0.0f, 1.0f)); // red color
      }
    }
    lManualObject->end();
  }

  // Now I attach it to a scenenode, so that it becomes present in the scene.
  Ogre::SceneNode* lNode = _sceneMgrPtr->getRootSceneNode()->createChildSceneNode();
  lNode->attachObject(lManualObject);

  ROS_INFO("Mesh Attached !");

  return true;
}

bool ViewerOgre::loadUnicMesh(Ogre::ManualObject *obj)
{
  if (! _isOgreInit)
  {
    ROS_ERROR("Viewer not init !");
    return false;
  }

  ROS_INFO("Loading Mesh");

  // Now I attach it to a scenenode, so that it becomes present in the scene.
  Ogre::SceneNode* lNode = _sceneMgrPtr->getRootSceneNode()->createChildSceneNode();
  lNode->attachObject(obj);

  ROS_INFO("Mesh Attached !");

  return true;
}

bool ViewerOgre::update(sensor_msgs::PointCloud2& obj)
{
  if (! _isOgreInit)
  {
    ROS_ERROR("Viewer not init !");
    return false;
  }

  _updateMesh(obj);

  if ( !_oculus->isMagCalibrated() )
  {
    ROS_WARN("Magnetometer not calibrated. Look left/right/up/down to collect enough samples.");
  }

  _oculus->updateProjectionMatrices();
  _oculus->update();

  _windowPtr->update(false);
  _windowPtr->swapBuffers(true);
  _rootPtr->renderOneFrame();

  _messagePump();

  return true;
}

bool ViewerOgre::update()
{
  if (! _isOgreInit)
  {
    ROS_ERROR("Viewer not init !");
    return false;
  }

  if ( !_oculus->isMagCalibrated() )
    ROS_WARN("Magnetometer not calibrated. Look left/right/up/down to collect enough samples.");

  _oculus->updateProjectionMatrices();
  _oculus->update();

  _windowPtr->update(false);
  _windowPtr->swapBuffers(true);
  _rootPtr->renderOneFrame();

  _messagePump();

  return true;
}

void ViewerOgre::_messagePump()
{
  if (!_isOgreInit)
    return;

  Ogre::WindowEventUtilities::messagePump();
}

bool ViewerOgre::isOgreInit()
{
  return _isOgreInit && _oculus->isOgreReady();
}

bool ViewerOgre::isInit()
{
  return isOgreInit() && _oculus->isOculusReady();
}

bool ViewerOgre::_initOgre(Ogre::String path, bool win)
{
  ROS_INFO("Init Ogre");
  _rootPtr.reset(new Ogre::Root(""));

  Strings lPluginNames;
  lPluginNames.push_back("/usr/lib/i386-linux-gnu/OGRE-1.7.4/RenderSystem_GL");
  lPluginNames.push_back("/usr/lib/i386-linux-gnu/OGRE-1.7.4/Plugin_ParticleFX");
  lPluginNames.push_back("/usr/lib/i386-linux-gnu/OGRE-1.7.4/Plugin_OctreeSceneManager");
  {
    Strings::iterator lIter = lPluginNames.begin();
    Strings::iterator lIterEnd = lPluginNames.end();

    for(;lIter != lIterEnd; lIter++)
    {
      Ogre::String& lPluginName = (*lIter);
      _rootPtr->loadPlugin(lPluginName);
    }
  }

  ROS_INFO("Init Renderer System");
  try
  {
    const Ogre::RenderSystemList& lRenderSystemList = _rootPtr->getAvailableRenderers();
    if( lRenderSystemList.size() == 0 )
      return false;

    Ogre::RenderSystem *lRenderSystem = lRenderSystemList[0];
    _rootPtr->setRenderSystem(lRenderSystem);
  }
  catch (Ogre::Exception &e)
  {
    ROS_ERROR(e.getDescription().c_str());
    return false;
  }

  ROS_INFO("Init Window");
  try
  {
    bool lCreateAWindowAutomatically = win;
    Ogre::String lWindowTitle = "oculus";
    Ogre::String lCustomCapacities = "";
    _rootPtr->initialise(lCreateAWindowAutomatically,
                         lWindowTitle, lCustomCapacities);
  }
  catch (Ogre::Exception &e)
  {
    ROS_ERROR(e.getDescription().c_str());
    return false;
  }

  if ( !_rootPtr->isInitialised() )
  {
    ROS_ERROR("OGRE isn't initialised !");
    return false;
  }

  ROS_INFO("Init Scene Manager");
  try
  {
    _sceneMgrPtr.reset( _rootPtr->createSceneManager(Ogre::ST_GENERIC, "SceneMgr") );
  }
  catch (Ogre::Exception &e)
  {
    ROS_ERROR(e.getDescription().c_str());
    return false;
  }

  ROS_INFO("Init Render Window");
  try
  {
    Ogre::String lWindowTitle = "Hello Ogre World";
    unsigned int lSizeX = 800;
    unsigned int lSizeY = 600;
    bool lFullscreen = false;
    Ogre::NameValuePairList lParams;
    lParams["FSAA"] = "0";
    lParams["vsync"] = "true";

    Ogre::RenderWindow *win = _rootPtr->createRenderWindow(lWindowTitle,
                                                           lSizeX, lSizeY,
                                                           lFullscreen, &lParams);

    if (win == NULL)
    {
      ROS_ERROR("Failed to initialise render window");
      return false;
    }

    _windowPtr.reset( win );
    _windowPtr->setVisible(true);
    _windowPtr->setActive(true);
    _windowPtr->setAutoUpdated(false);
  }
  catch (Ogre::Exception &e)
  {
    ROS_ERROR(e.getDescription().c_str());
    return false;
  }

  bool init = false;

  ROS_INFO("Init Oculus");
  init += _oculus->setupOculus();

  ROS_INFO("Init Oculus Ogre");
  init += _oculus->setupOgre(_sceneMgrPtr.get(), _windowPtr.get(), NULL, path);

  return init;
}

void ViewerOgre::_updateMesh(sensor_msgs::PointCloud2& obj)
{
  ROS_DEBUG("Updating Mesh");

  Ogre::ManualObject * mObj = _sceneMgrPtr->getManualObject(_meshName);

  sensor_msgs::PointCloud2Iterator<float> iter_x(obj, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(obj, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(obj, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(obj, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(obj, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(obj, "b");

  mObj->beginUpdate(0);
  {
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z,
                                   ++iter_r, ++iter_g, ++iter_b)
    {
      // populate with some data
      mObj->position(*iter_x, *iter_y, *iter_z);// a vertex
      mObj->colour(Ogre::ColourValue(((float)(*iter_r))/255.f,
                                     ((float)(*iter_g))/255.f,
                                     ((float)(*iter_b))/255.f, 1.0f)); // green color
    }
  }
  mObj->end();

  ROS_DEBUG("Mesh Updated !");
}
