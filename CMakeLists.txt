cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#Uses PCL
find_package( PCL REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )

rosbuild_add_executable( turtlebot_teleop src/turtlebot_key.cpp )
target_link_libraries( turtlebot_teleop tinyxml )

rosbuild_add_executable( turtlebot_wander src/turtlebot_wander.cpp )
target_link_libraries( turtlebot_wander tinyxml )
# Image Processing - OpenCV
rosbuild_add_executable( grabImage src/grabImage.cpp )
# Pointcloud Processing - PCL
rosbuild_add_executable( grabPointcloud src/grabPointcloud.cpp ) 
# Bumps
rosbuild_add_executable( grabBumps src/grabBumps.cpp ) 
# Move it baby
rosbuild_add_executable( moveRobot src/moveRobot.cpp src/robotController.cpp )
