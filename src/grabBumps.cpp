/**
 * @file grabBumps.cpp
 */
#include <ros/ros.h>
#include <turtlebot_node/TurtlebotSensorState.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <iostream>

/** Functions declaration */
void init();
void publish( );

/** Global variables */
ros::NodeHandle *gNh;
ros::Subscriber gSh;
ros::Publisher gCliffPub;

  uint8_t gBumpsWheeldrops;
  uint8_t gWall;

  uint8_t gCliffRight;
	uint8_t gCliffLeft;
	uint8_t gCliffFrontLeft;
	uint8_t gCliffFrontRight;

  uint16_t gCliffLeftSignal;
	uint16_t gCliffRightSignal;
	uint16_t gCliffFrontLeftSignal;
	uint16_t gCliffFrontRightSignal;

	double gAngle;

/**
 * @function bumpers_cb
 * @brief Callback
 */
void bumpers_cb( const turtlebot_node::TurtlebotSensorState::ConstPtr& _input ) {

	gCliffRight= _input->cliff_right;
	gCliffLeft= _input->cliff_left;
	gCliffFrontLeft = _input->cliff_front_left;
	gCliffFrontRight = _input->cliff_front_right;

  gBumpsWheeldrops = _input->bumps_wheeldrops;
  gWall = _input->wall;
	gAngle = _input->angle;
//	ROS_INFO("Cliff right : %d left: %d front left: %d front right: %d \n", gCliffRight, gCliffLeft, gCliffFrontRight, gCliffFrontLeft );
//	ROS_INFO("Wheeldrops: %d wall: %d \n", gBumpsWheeldrops, gWall);
	ROS_INFO("Angle: %.3f \n", gAngle);
}

/**
 * @function main
 */
int main( int argc, char** argv ) {

  // Initialize ROS
  ros::init( argc, argv, "grabBumps" );
	init();

  ros::Rate loop_rate(10);

  ROS_INFO("Banner again ! \n");

  /** Loop */
  while( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
    publish();
  }
      
  return(0);
}

/**
  * @function init
  */
void init() {
	gNh = new ros::NodeHandle();
  gSh = gNh->subscribe( "input", 1, bumpers_cb );
  //gCliffPub = gNh->advertise<uint8_t>("output", 1);
}

/**
  * @function publish
  */
void publish() {
	//gCliffPub.publish( gCliffRight);
}
