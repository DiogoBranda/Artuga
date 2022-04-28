/**

  @file camera.h
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


#ifndef CAMERA_H
#define CAMERA_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


/**
 * @brief - The camera class contains the camera parameters values and distortion coefficients.
 * Furthermore it provides the image capture by the camera.
 */
class camera
{
  private:

    char config = 0;// camera parameters loaded
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    char imageReady =0; // image loaded
    cv_bridge::CvImagePtr image;

  public:
    camera();

    /**
     * @brief cam_par - callback function use to save the camera parameters
     * @param camInfo_msg
     */
    void cb_camPar (const sensor_msgs::CameraInfoConstPtr& camInfo_msg);

    /**
     * @brief cam_img - callback function use to save the image provides by the camera
     * @param image_msg
     */
    void cb_camImg(const sensor_msgs::CompressedImage::ConstPtr & image_msg);

    /**
     * @brief getCameraMatrix - provides the camera parameters
     * @return cv::Mat type matrix with the camera parameters
     */
    cv::Mat getCameraMatrix();

    /**
     * @brief getDistCoeffs - provides the camera distortion coefficients
     * @return cv::Mat type matrix with the camera distortion coefficients
     */
    cv::Mat getDistCoeffs();

    /**
     * @brief getImage - provides the image caped from the camera
     * @return  cv_bridge::CvImagePtr type variable  with the image caped from the camera
     */
    cv_bridge::CvImagePtr getImage();

    /**
     * @brief getImageReady - If the camera is capting image returns 1. If not it returns 0.
     * @return char type variable
     */
    char getImageReady();

    /**
     * @brief clearImageReady - Set imageReady variable to 0.
     */
    void clearImageReady();

    /**
     * @brief getImageReady - If the camera parameters are ready returns 1. If not it returns 0.
     * @return char type variable
     */
    char getConfigReady();
};

#endif // CAMERA_H
