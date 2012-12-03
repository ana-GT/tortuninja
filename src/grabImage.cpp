/**
 * @file grabImage.h
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "imageProcessing/imageProcessing.h"

/**
 * @function main
 */
int main( int argc, char** argv ) {
  ros::init( argc, argv, "grabImage" );
  imageProcessing ip;
  ros::Rate r(10);

  while( ros::ok() ) {
    ros::spinOnce();
    r.sleep();
    //ip.update();
  }

  return 0;
}
