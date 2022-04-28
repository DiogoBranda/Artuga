/**

  @file camera.cpp
  @author Diogo Brandão Silva
  @brief Camera setup with ROS
  @version 1.1
  @date 28-04-2022

  MIT License

  Copyright (c) 2022 Diogo Brandão

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

**/

/*#####################
  ##     Includes    ##
  #####################*/

#include "camera.h"



camera::camera()
{

}

/**
 * @brief cam_par - callback function used to save the camera parameters
 * @param camInfo_msg
 */
void camera::cb_camPar (const sensor_msgs::CameraInfoConstPtr& camInfo_msg){
  // If not config (No parameters set before)
  if(this->config==0){
    cv::Mat cameraMatrix_pt(3, 3, CV_64FC1, (void *) camInfo_msg->K.data());
    cv::Mat distCoeffs_pt(1,5, CV_64FC1, (void *) camInfo_msg->D.data());
    cameraMatrix_pt.copyTo( this->cameraMatrix);
    distCoeffs_pt.copyTo(this->distCoeffs);
    this->config=1; // config done
  }
}

/**
 * @brief cam_img - callback function use to save the image provides by the camera
 * @param image_msg
 */
void camera::cb_camImg(const sensor_msgs::CompressedImage::ConstPtr & image_msg){
  try{
    this->image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    this->imageReady = 1;
  }

  catch (cv_bridge::Exception& e){
    return;
  }
}

/**
 * @brief getCameraMatrix - provides the camera parameters
 * @return cv::Mat type matrix with the camera parameters
 */
cv::Mat camera::getCameraMatrix(){
  return this->cameraMatrix;
}

/**
 * @brief getDistCoeffs - provides the camera distortion coefficients
 * @return cv::Mat type matrix with the camera distortion coefficients
 */
cv::Mat camera::getDistCoeffs(){
  return this->distCoeffs;
}
/**
 * @brief getImage - provides the image caped from the camera
 * @return  cv_bridge::CvImagePtr type variable  with the image caped from the camera
 */
cv_bridge::CvImagePtr camera::getImage(){
  return  this->image;
}
/**
 * @brief getImageReady - If the camera is capting image returns 1. If not it returns 0.
 * @return char type variable
 */
char camera::getImageReady(){
  return this->imageReady;
}
/**
 * @brief clearImageReady - Set imageReady variable to 0.
 */
void camera::clearImageReady(){
   this->imageReady=0;
}

/**
 * @brief getImageReady - If the camera parameters are ready returns 1. If not it returns 0.
 * @return char type variable
 */
char camera::getConfigReady(){
  return this->config;
}
