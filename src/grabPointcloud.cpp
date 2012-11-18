/**
 * @file grabPointcloud.cpp
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Display stuff - PCL 1.5 in ROS, some things are missing, bang! :(
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <boost/thread.hpp>

ros::Publisher pub;
boost::shared_ptr<pcl::visualization::CloudViewer> viewer( new pcl::visualization::CloudViewer("3D Viewer") );

/**
 * @function cloud_cb
 * @brief Callback
 */
void cloud_cb( const sensor_msgs::PointCloud2ConstPtr& _input ) {

  // Convert to PCL pointcloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );

  pcl::fromROSMsg( *_input, *cloud );
  // Show it
  viewer->showCloud( cloud );

  // .. Do data processing
  sensor_msgs::PointCloud2 output;
  // Publish the data
  pub.publish( output );
}

/**
 * @function main
 */
int main( int argc, char** argv ) {

  // Initialize ROS
  ros::init( argc, argv, "grabPointcloud" );
  ros::NodeHandle nh;

  // Create viewer
  

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe( "input", 1, cloud_cb );
  
  // Create a ROS publisher for the output PCL pointcloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Keep open
  while( !viewer->wasStopped() ) {

    
    boost::this_thread::sleep( boost::posix_time::seconds(1) );
    // Spin
    ros::spinOnce();
  }
}
