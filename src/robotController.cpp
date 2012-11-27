/**
 * @file robotController.cpp
 * @author A. Huaman
 */
#include "robotController.h"

/**
 * @function robotController
 */
robotController::robotController() {

  mMinLeftIndex = 157 - 5; //179 - 5;
  mMaxLeftIndex = 157; // 179
  mMinRightIndex = 22;
  mMaxRightIndex = 22 + 5;
  mMinFrontIndex = 90 - 5;
  mMaxFrontIndex = 90 + 5;

  mMinThreshDist = 0.10;
  mMaxThreshDist = 0.15;
  mThreshFrontCloseToWall = 0.6;

  mRefAngle = 22.0*3.1416/180.0;
  mRefCos = cos(mRefAngle);
  mBehavior =  GET_CLOSE_LEFT_WALL;
}

/**
 * @function ~robotController
 */
robotController::~robotController() {

  if( mNode_private != NULL ) {
    delete mNode_private;
  }
  if( mNode != NULL ) {
    delete mNode;
  }
}

/**
 * @function init
 */
void robotController::init() {

  mNode_private = new ros::NodeHandle("~");
  mNode = new ros::NodeHandle("");

  mLinear_Vel = 1.0;
  mAngular_Vel = 0.5;
  mLinear_Scale = 1.0;
  mAngular_Scale = 1.0;

  mMode = 0; // Clockwise (CW)

  mNode_private->param( "scale_angular", mAngular_Scale, mAngular_Scale );
  mNode_private->param( "scale_linear", mLinear_Scale, mLinear_Scale );
  mNode_private->param( "mode", mMode, mMode );
  
  mPublisher_Vel = mNode->advertise<geometry_msgs::Twist>("cmd_vel", 1);
}


/**
 * @function update
 */
void robotController::update() {
  publish( mAngular_Vel, mLinear_Vel );
}

/**
 * @function publish
 */
void robotController::publish( double _angular, 
			       double _linear ) {
  
  geometry_msgs::Twist vel;
  
  vel.angular.z = mAngular_Scale*_angular;
  vel.linear.x = mLinear_Scale*_linear;
  
  mPublisher_Vel.publish(vel);  
  
  return;
}

/**
 * @function scan_callback
 */
void robotController::scan_callback( const sensor_msgs::LaserScanConstPtr& _scan ) {
  
  mRangeMin = _scan->range_min;
  mRangeMax = _scan->range_max;


  // Get distance info
  getDistanceData( _scan );
  //printDistanceInfo();
}

/**
 * @function getDistanceData
 */
void robotController::getDistanceData( const sensor_msgs::LaserScanConstPtr& _scan ) {

  float dist;
  float dist_i;
  float numValidDist;

  // Left distance
  dist = 0;
  numValidDist = 0;

  for( int i = mMinLeftIndex; i <= mMaxLeftIndex; ++i ) {

    dist_i = _scan->ranges[i];
    if( dist_i > mRangeMin && dist_i < mRangeMax ) {
      dist = dist + dist_i;
      numValidDist++;
    } 
  }
  if( numValidDist > 0 ) { mLeftDist = (dist / numValidDist)*mRefCos; }
  else { mLeftDist = mMinThreshDist; printf("[getDistanceData] Using default thresh for left dist: No valid scan input \n"); }
  

  // Right distance
  dist = 0;
  numValidDist = 0;

  for( int i = mMinRightIndex; i <= mMaxRightIndex; ++i ) {

    dist_i = _scan->ranges[i];
    if( dist_i > mRangeMin && dist_i < mRangeMax ) {
      dist = dist + dist_i;
      numValidDist++;
    } 
  }
  if( numValidDist > 0 ) { mRightDist = (dist / numValidDist)*mRefCos; }
  else { mRightDist = mMinThreshDist; printf("[getDistanceData] Using default thresh for right dist: No valid scan input \n"); }


  // Front distance
  dist = 0;
  numValidDist = 0;

  for( int i = mMinFrontIndex; i <= mMaxFrontIndex; ++i ) {

    dist_i = _scan->ranges[i];
    if( dist_i > mRangeMin && dist_i < mRangeMax ) {
      dist = dist + dist_i;
      numValidDist++;
    } 
  }
  if( numValidDist > 0 ) { mFrontDist = dist / numValidDist; }
  else { mFrontDist = mMinThreshDist; printf("[getDistanceData] Using default thresh for front dist: No valid scan input \n"); }


}

/**
 * @function printDistanceInfo
 */
void robotController::printDistanceInfo() {
  printf( "* Distance to the Left: %.3f \n", mLeftDist );
  printf( "* Distance to the Right: %.3f \n", mRightDist );
  printf( "* Distance to the Front: %.3f \n", mFrontDist );
}

/**
 * @function updateBehavior
 */
void robotController::updateBehavior() {
  printf("updateBehavior \n");
  /*  if( mLeftDist > mMinThreshDist ) {
    mBehavior = GET_CLOSE_LEFT_WALL;
    }*/

  if( mBehavior == GET_CLOSE_LEFT_WALL ) {
    getCloseLeftWall();
  }
  else if( mBehavior == STAY_QUIET ){
    mLinear_Vel = 0;
    mAngular_Vel = 0;
  }
}

/**
 * @function getCloseLeftWall
 */
void robotController::getCloseLeftWall() {

  ros::Rate loop_rate(10);
  
  float prev_dist;
  float current_dist;

  // Face the wall
  printf("Start Face the wall \n");
  prev_dist =  1001;
  current_dist = 1000;

  do {
    if( mFrontDist > mRangeMin && 
	mFrontDist < mRangeMax && 
	mFrontDist < mThreshFrontCloseToWall ) {
      prev_dist = current_dist;
      current_dist = mFrontDist;
    }

    mAngular_Vel = +1*0.5;
    mLinear_Vel = 0.0;

    update();
    ros::spinOnce();
    loop_rate.sleep();
    printf("loop face the wall: prev: %f current: %f \n", prev_dist, current_dist);
  } while( prev_dist > current_dist );
  
  // Walk until you are close enough
  printf("Walk in front until you are near enough the wall \n");
  do {
    mLinear_Vel = 0.1;
    mAngular_Vel = 0.0;

    update();
    ros::spinOnce();
    loop_rate.sleep();
    printf("mFrontDist: %f \n", mFrontDist);
  } while( mFrontDist > 0.35 );


    mAngular_Vel = 0.0;
    mLinear_Vel = 0.0;

    update();  
  
  // Positionate parallel to the wall
  printf("Positionate parallel to wall \n");
  prev_dist = 1001;
  current_dist = 1000;
  do {
    if( mLeftDist > 0 && mLeftDist < mThreshFrontCloseToWall ) {
      prev_dist = current_dist;
      current_dist = mLeftDist;
      printf("Loop: prev dist: %f curre dist: %f \n", prev_dist, current_dist);
    }

    mAngular_Vel = -1*0.5;
    mLinear_Vel = 0.0;

    update();
    ros::spinOnce();
    loop_rate.sleep();
  } while( prev_dist > current_dist );
  printf("Ready for next behavior: Left distance from wall: %f current dist: %f \n", mLeftDist, current_dist);
  
  printf("Stay quiet \n");
  // Stay quiet until next order
  mBehavior = STAY_QUIET;
}

/**
 * @function wallFollowing
 */
void robotController::wallFollowing() {

  // Wall Left
  if( mMode == 0 ) {
    
    
    // If nothing ahead
    if( mFrontDist > mMinThreshDist ) {

      // If too close to the wall, rotate away
      if( mLeftDist < mMinThreshDist ) {
	mAngular_Vel = -1*0.5;
	mLinear_Vel = 0.0;
	printf("Rotate away from left wall - Left dist: %f  \n", mLeftDist);
      }
      
      // If too far, rotate near
      else if( mLeftDist > mMaxThreshDist ) {
	mAngular_Vel = +1*0.5;
	mLinear_Vel = 0.1;
	printf("Rotate near to  left wall - Left dist: %f \n", mLeftDist);
      }
      
      // If it is not so close yet not that far, keep ahead
      else {
	mAngular_Vel = 0.0;
	mLinear_Vel = 0.5;
	printf("Go forward - Left Dist: %f  \n", mLeftDist );
      }
      
    } // Front Dist
    else {
      mAngular_Vel = -1*0.5;
      mLinear_Vel = 0.0;
      printf("Getting away from front wall - Front wall dist: %f \n", mFrontDist );
    }
    
  } // Wall left
    
}
  
