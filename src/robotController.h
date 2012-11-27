/**
 * @file robotController.h
 * @author A. Huaman
 */

#ifndef _ROBOT_CONTROLLER_TURTLEBOT_
#define _ROBOT_CONTROLLER_TURTLEBOT_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

/**  BEHAVIORS */
typedef enum BEHAVIORS{
  GET_CLOSE_LEFT_WALL = 5,
  GET_CLOSE_RIGHT_WALL,
  STAY_QUIET
};

/**
 * @class robotController
 */
class robotController {

 public:
  robotController();
  ~robotController();
  void init();
  void publish( double _angular, 
		double _linear );
  void update();
  void scan_callback( const sensor_msgs::LaserScanConstPtr& _scan );
  void getDistanceData( const sensor_msgs::LaserScanConstPtr& _scan ) ;
  void printDistanceInfo();
  void wallFollowing();

  void updateBehavior();
  void getCloseLeftWall();

 private:

  ros::NodeHandle *mNode, *mNode_private;
  double mLinear_Vel, mAngular_Vel;
  double mLinear_Scale, mAngular_Scale;
  ros::Publisher mPublisher_Vel;
  ros::Subscriber mSubscriber;
  
  int mMode; // 0: CW, 1: CCW

  float mLeftDist;
  float mRightDist;
  float mFrontDist;

  int mMinLeftIndex;
  int mMaxLeftIndex;
  int mMinRightIndex;
  int mMaxRightIndex;
  int mMinFrontIndex;
  int mMaxFrontIndex;

  float mRangeMin;
  float mRangeMax;
  float mRefAngle;
  float mRefCos;

  int mBehavior;

  // Constant stuff
  float mMinThreshDist;
  float mMaxThreshDist;
  float mThreshFrontCloseToWall;

};

#endif /** _ROBOT_CONTROLLER_TURTLEBOT_ */
