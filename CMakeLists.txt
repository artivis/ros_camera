cmake_minimum_required(VERSION 2.8.3)
project(ros_camera_rig)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
    image_transport
    ros_img_sync
)

find_package( OpenCV REQUIRED )

# Oculus SDK
FIND_PACKAGE( OculusSDK REQUIRED )
link_directories(${OculusSDK_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    image_transport
    ros_img_sync
  DEPENDS OculusSDK
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OGRE OGRE)

execute_process(COMMAND
pkg-config --libs OGRE
OUTPUT_VARIABLE OGRE_LIBRARIES
OUTPUT_STRIP_TRAILING_WHITESPACE
)

###########
## Build ##
###########

include_directories(
   ${catkin_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${OculusSDK_INCLUDE_DIRS} include
)

add_library(${PROJECT_NAME}
                           src/image_handler.cpp
                           src/camera_handler.cpp
                           src/camera_rig.cpp
                           src/usefull.cpp
                           include/ros_camera_rig/impl/point_cloud2_iterator.h
                           src/ogre_oculus.cpp
                           src/ogre_view.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OculusSDK_LIBRARIES} ${OGRE_LIBRARIES})

add_executable(demo_spherical_pointcloud sample/demo_spherical_pointcould.cpp)
target_link_libraries(demo_spherical_pointcloud
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

add_executable(demo_oculus sample/demo_oculus.cpp)
target_link_libraries(demo_oculus
   ${catkin_LIBRARIES} ${OculusSDK_LIBRARIES} ${OGRE_LIBRARIES} ${PROJECT_NAME}
)

#add_executable(demo_spherical_panorama sample/demo_spherical_panorama.cpp)
#target_link_libraries(demo_spherical_panorama
#   ${catkin_LIBRARIES} ${PROJECT_NAME}
#)

