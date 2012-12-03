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

  mMode = 0; 

  mNode_private->param( "scale_angular", mAngular_Scale, mAngular_Scale );
  mNode_private->param( "scale_linear", mLinear_Scale, mLinear_Scale );
  mNode_private->param( "mode", mMode, mMode );
  
  mSubscriber = mNode->subscribe( "odom", 1, &robotController::odometry_callback, this );
  mPublisher_Vel = mNode->advertise<geometry_msgs::Twist>("cmd_vel", 1);
}


/**
 * @function update
 */
void robotController::update() {
  publish( mAngular_Vel, mLinear_Vel );
  printDistanceInfo();
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

/////////////////// CALLBACKS /////////////////////
/**
 * @function odometry_callback
 */
void robotController::odometry_callback( const nav_msgs::Odometry& _msg ) {

  mPosX = _msg.pose.pose.position.x;
  mPosY = _msg.pose.pose.position.y;
  mPosZ = _msg.pose.pose.position.z;

  mOrient = 2*atan2(  _msg.pose.pose.orientation.z, _msg.pose.pose.orientation.w );

}

/**
 * @function scan_callback
 * @brief
 */
void robotController::scan_callback( const sensor_msgs::LaserScanConstPtr& _scan ) {
  
  mRangeMin = _scan->range_min;
  mRangeMax = _scan->range_max;


  // Get distance info
  getDistanceData( _scan );
  //printDistanceInfo();
}

///////////////////////////////////////////////////////

/**
 * @function rotate
 * @brief
 */
bool robotController::rotate( float _angle, float _angularSpeed ) {

  float startOrientation;
  ros::Rate loop_rate(10);
  bool flag;

  // Read initial value
  ros::spinOnce();
  startOrientation = mOrient;

  // Set only rotation
  mLinear_Vel = 0;
  if( _angle > 0 ) { mAngular_Vel = _angularSpeed; }
  else { mAngular_Vel = -_angularSpeed; }

  // Rotate until you got very close to the rotation goal
  flag = true;

  do {    
    update();
    ros::spinOnce();
    loop_rate.sleep();

    if( _angle > 0 ) {
      if( mOrient - startOrientation < _angle ) { flag = true; }
      else { flag = false; }
    }
    else {
      if( mOrient - startOrientation > _angle ) { flag = true; }
      else { flag = false; }
    }
  } while( flag == true  ); 

  // Stop the robot
  mLinear_Vel = 0.0;
  mAngular_Vel = 0.0;
  update();

  return true;
}

/**
 * @function goStraight
 * @brief
 */
bool robotController::goStraight( float _distance, float _linearSpeed ) {

  float startPosX, startPosY;
  float dist;
  float distanceMod;
  ros::Rate loop_rate(10);
  bool flag;

  // Read initial value
  ros::spinOnce();
  startPosX = mPosX;
  startPosY = mPosY;

  // Set only translation
  mAngular_Vel = 0.0;
  if( _distance > 0 ) { mLinear_Vel = _linearSpeed; distanceMod = _distance;  }
  else { mLinear_Vel = -_linearSpeed; distanceMod = -1*_distance; }

  // Go forward until you reach your goal
  flag = true;

  // Go straight
  do {    
    update();
    ros::spinOnce();
    loop_rate.sleep();

    dist = sqrt( (mPosX - startPosX)*(mPosX - startPosX) + (mPosY - startPosY)*(mPosY - startPosY) );

    if( dist < distanceMod ) { flag = true; }
    else { flag = false; }
    
    printf("Start x: %f pos x: %f distance: %f, diff: %f \n", startPosX, mPosX, _distance, mPosX - startPosX );
  } while( flag == true  ); 


  // Stop the robot
  mLinear_Vel = 0.0;
  mAngular_Vel = 0.0;
  update();

  return true;
}

/**
 * @function approach
 * @brief
 */
void robotController::approach( float _dist, float _danceDist, int _numberSteps ) {
  
  // Go straight
  goStraight( _dist - _danceDist, 0.5 );
  // Dance 
  for( unsigned int i = 0; i < _numberSteps; ++i ) {
    goStraight( _danceDist );
    goStraight( -1*_danceDist );
  }
  
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

  // Following wall left
  if( mMode == 0 ) {
    //peekaboo_left( 0.3, 1.57, 5 );
  }
  // Following wall right
  else if( mMode == 1 ) {
    //peekaboo_right( 0.3, 1.57, 5 );
  }
  approach( 1.0, 0.3, 3 );
}

/**
 * @function peekaboo_right
 * @brief Walk - look - walk - look
 */
void robotController::peekaboo_right( float _lengthStride, float _visionAngle, int _numStrides ) {
  printf("Start peekaboo right\n");
  for( unsigned int i = 0; i < _numStrides; ++i ) {
    goStraight( _lengthStride );
    rotate( -1*_visionAngle );
    rotate( _visionAngle );
  }
  printf("[peekaboo_right] Finished number of strides: %d  \n", _numStrides);
}

/**
 * @function peekaboo_left
 * @brief Walk - look - walk - look
 */
void robotController::peekaboo_left( float _lengthStride, float _visionAngle, int _numStrides ) {
  printf("Start peekaboo left\n");
  for( unsigned int i = 0; i < _numStrides; ++i ) {
    goStraight( _lengthStride );
    rotate( _visionAngle );
    rotate( -1*_visionAngle );
  }
  printf("[peekaboo_left] Finished number of strides: %d  \n", _numStrides);
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
  
