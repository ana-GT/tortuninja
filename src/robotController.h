/**
 * @file robotController.h
 * @author A. Huaman
 */

#ifndef _ROBOT_CONTROLLER_TURTLEBOT_
#define _ROBOT_CONTROLLER_TURTLEBOT_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
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
  void odometry_callback( const nav_msgs::Odometry& _msg );

  void getDistanceData( const sensor_msgs::LaserScanConstPtr& _scan ) ;
  void printDistanceInfo();
  void wallFollowing();

  void updateBehavior();
  void getCloseLeftWall();

  bool rotate( float _angle, float _angularSpeed = 0.5 );
  bool goStraight( float _distance, float _linearSpeed = 0.5 );
  void approach( float _dist, float _danceDist = 0.3, int _numberSteps = 2 );
  void peekaboo_left( float _lengthStride = 0.1, float _visionAngle = 1.57, int _numStrides = 4 );
  void peekaboo_right( float _lengthStride = 0.1, float _visionAngle = 1.57, int _numStrides = 4 ); 

 private:

  ros::NodeHandle *mNode, *mNode_private;
  double mLinear_Vel, mAngular_Vel;
  double mLinear_Scale, mAngular_Scale;
  ros::Publisher mPublisher_Vel;
  ros::Subscriber mSubscriber;
  
  int mMode; // 0: Left, 1: Right

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

  // Robot state
  float mPosX, mPosY, mPosZ;
  float mOrient;

};

#endif /** _ROBOT_CONTROLLER_TURTLEBOT_ */
