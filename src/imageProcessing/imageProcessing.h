/**
 * @class imageProcessing
 * @brief
 */

#ifndef _IMAGE_PROCESSING_H_
#define _IMAGE_PROCESSING_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace enc = sensor_msgs::image_encodings;

/**
 * @class imageProcessing
 */
class imageProcessing {

  ros::NodeHandle mNh;
  image_transport::ImageTransport mIt;
  image_transport::Subscriber mImage_Sub;
  image_transport::Publisher mImage_Pub;
  std::string mWindowName;
  cv::Mat mCurrentImage;
  cv::Mat mBlobImage;

  public:
  imageProcessing();
  ~imageProcessing();

  void imageCallback( const sensor_msgs::ImageConstPtr& msg );
  void blobFinder( cv::Mat _image );

}; /** Image Processing */

#endif /** _IMAGE_PROCESSING_H_ */
