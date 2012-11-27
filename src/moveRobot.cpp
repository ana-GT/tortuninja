/**
 * @file moveRobot.cpp
 */
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "robotController.h"

robotController gRc;

/**
 * @function main
 */
int main( int argc, char** argv ) {

  ros::init(argc, argv, "moveRobot");

  gRc.init();
  ros::NodeHandle node_;
  ros::Subscriber subscriber_ = node_.subscribe( "scan", 1, &robotController::scan_callback, &gRc );

  ros::Rate loop_rate(10);

  /** Loop */
  while( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
    gRc.update( );
    gRc.updateBehavior();
  }
      
  return(0);
}


