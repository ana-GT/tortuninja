/**
 * @file imageProcessing.cpp
 * @brief
 * @author A. Huaman (based on ROS documentation sample which I don't remember exactly)
 */
#include "imageProcessing.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <cvblob.h>

using namespace std;
using namespace cvb;

/**
 * @function imageProcessing
 * @brief
 */
imageProcessing::imageProcessing(): mIt( mNh ) {
  std::string ns( ros::this_node::getNamespace() );
  char completeName[60];
  sprintf( completeName, "%s_Robot_Watcher", ns.c_str() );

  mWindowName = completeName;
  mImage_Sub = mIt.subscribe( "in", 1, &imageProcessing::imageCallback, this );
  
  cv::namedWindow( mWindowName );
}

/**
 * @function imageProcessing
 * @brief
 */
imageProcessing::~imageProcessing() {
  cv::destroyWindow( mWindowName );
}

/**
 * @function imageCallback
 * @brief
 */
void imageProcessing::imageCallback( const sensor_msgs::ImageConstPtr& msg ) {

  ROS_INFO("Got callback image \n");
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( msg, enc::BGR8 );
  } catch ( cv_bridge::Exception& e ) {
    ROS_ERROR(" cv_bridge exception: %s", e.what() );
  }

  ros::spinOnce();
  mCurrentImage = cv_ptr->image;
  if( mCurrentImage.rows > 320 ) {
    blobFinder( mCurrentImage );
    cv::imshow( mWindowName, mBlobImage );
    cv::waitKey( 3 );
  }
  
}


/**
 * @function blobFinder
 */
void imageProcessing::blobFinder( cv::Mat _image ) {

  IplImage *img = cvCreateImage( cvSize(_image.cols, _image.rows), 
				 8, 
				 _image.channels() );
  img->imageData = (char*) _image.data;

  //cvSetImageROI( img, cvRect( 0, 0, img->width - 1, img->height - 1) );

  IplImage *grey = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
  cvCvtColor(img, grey, CV_BGR2GRAY);
  cvThreshold(grey, grey, 100, 255, CV_THRESH_BINARY);

  IplImage *labelImg = cvCreateImage(cvGetSize(grey),IPL_DEPTH_LABEL,1);

  CvBlobs blobs;
  unsigned int result = cvLabel(grey, labelImg, blobs);

  IplImage *imgOut = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3); cvZero(imgOut);
  cvRenderBlobs(labelImg, blobs, img, imgOut);

  //unsigned int i = 0;

  // Render contours:
  for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
  {
    //cvRenderBlob(labelImg, (*it).second, img, imgOut);
    
    CvScalar meanColor = cvBlobMeanColor((*it).second, labelImg, img);
    cout << "Mean color: r=" << (unsigned int)meanColor.val[0] << ", g=" << (unsigned int)meanColor.val[1] << ", b=" << (unsigned int)meanColor.val[2] << endl;

    CvContourPolygon *polygon = cvConvertChainCodesToPolygon(&(*it).second->contour);

    CvContourPolygon *sPolygon = cvSimplifyPolygon(polygon, 10.);
    CvContourPolygon *cPolygon = cvPolygonContourConvexHull(sPolygon);

    cvRenderContourChainCode(&(*it).second->contour, imgOut);
    cvRenderContourPolygon(sPolygon, imgOut, CV_RGB(0, 0, 255));
    cvRenderContourPolygon(cPolygon, imgOut, CV_RGB(0, 255, 0));

    if( cPolygon ) { delete cPolygon; }
    if( sPolygon ) { delete sPolygon; }
    if( polygon ) { delete polygon; }

    // Render internal contours: 
    /*
      for (CvContoursChainCode::const_iterator jt=(*it).second->internalContours.begin(); jt!=(*it).second->internalContours.end(); ++jt)
	cvRenderContourChainCode((*jt), imgOut);
    */
  }

  // Copy to image out
  cv::Mat ap( imgOut );

  mBlobImage = ap.clone();

  cvReleaseImage(&imgOut);
  cvReleaseImage(&grey);
  cvReleaseImage(&labelImg);
  cvReleaseImage(&img);

  cvReleaseBlobs(blobs);

}
