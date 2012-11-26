/**
 * @file moveRobot
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

/** Functions declaration */
void init();
void publish( double _angular, 
	      double _linear );

/** Global variables */
ros::NodeHandle *gNh, *gPh;
double gLinear, gAngular;
double gLScale, gAScale;
ros::Publisher gVelPub;
int gMoveType;


/**
 * @function main
 */
int main( int argc, char** argv ) {

  ros::init(argc, argv, "moveRobot");

  init();

  ros::Rate loop_rate(10);

  /** Loop */
  while( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
    gLinear = 0.5; // 1.0
    gAngular = 0.5;
    publish( gAngular, gLinear );

    ROS_INFO( " Running! \n" );
  }
      
  return(0);
}


/**
 * @function init
 */
void init() {
  gPh = new ros::NodeHandle("~");
  gNh = new ros::NodeHandle("");
  gLinear = 0;
  gAngular = 0;
  gLScale = 1.0;
  gAScale = 1.0;
  gMoveType=0;
  gPh->param("scale_angular", gAScale, gAScale );
  gPh->param("scale_linear", gLScale, gLScale );

	gPh->param("move_type", gMoveType, gMoveType );
  gVelPub = gNh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

/**
 * @function publish
 */
void publish( double _angular, 
	      double _linear ) {

  geometry_msgs::Twist vel;

  if( gMoveType == 0 ) {
  	vel.angular.z = 0*gAScale*_angular;
  	vel.linear.x = gLScale*_linear;
  }
  else {
  	vel.angular.z = 0*-1*gAScale*_angular;
  	vel.linear.x = -1*gLScale*_linear;
	}
  gVelPub.publish(vel);    
  
  return;
}



