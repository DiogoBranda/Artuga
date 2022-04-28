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


#ifndef ARTUGAMARKER_H
#define ARTUGAMARKER_H

#include <string>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include<eigen_conversions/eigen_msg.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>

class artugaMarker
{
  private:
    int id;
    float size;
    geometry_msgs::PoseStamped poseAruco;
    bool state;


  public:

    /**
     * @brief arucoCras  - Class constructor
     * @param id - id of aruco (INT)(in)
     * @param size - size of aruco (float)(in)
     */
    artugaMarker(int id, float size);

    /**
     * @brief setArucoPose - set aruco pos with the values post on the function
     * @param x - x cordenate value (float)(in)
     * @param y - y cordenate value (float)(in)
     * @param z - z cordenate value (float)(in)
     * @param state - state 1 if aruco is detect 0 if not (bool)(in)
     * @param Q1 - Quaternion value (Eigen::Quaterniond)(in)
     */
    void setArucoPose(float x, float y, float z, bool state,const Eigen::Quaterniond &Q1);

    /**
     * @brief getArucoPose - provides the pose of aruco relative to something
     * @return returns the aruco pose (geometry_msgs::PoseStamped)
     */
    geometry_msgs::PoseStamped getArucoPose();

    /**
     * @brief getId - provides the aruco id
     * @return aruco id (int)
     */
    int getId();

    /**
     * @brief getSize - provides the aruco size
     * @return aruco size (float)
     */
    float getSize();


    /**
     * @brief getState - provides the aruco state
     * @return aruco state, 0 if not detect 1 if is detect (bool)
     */
    bool getState();

    /**
     * @brief detector given an image it detects the arucos an estimate the position,
     * then stores on the variable aruco passe on the function.
     * @param img - image to try detect the arucos (cv::Mat)(in)
     * @param aruco - aruco variable where the positions of arucos that where detect will be stored (arucoCras **)(out)
     * @param n - number of arucos on that exist (int)(in)
     * @param cameraMatrix - camera matrix (cv::Mat)(in)
     * @param distCoeffs -  camera distorcion values (cv::Mat)(in)
     */
    static void detector(cv::Mat img, artugaMarker **aruco, int n, cv::Mat cameraMatrix, cv::Mat distCoeffs ,std::string display,bool flag);

};

#endif // ARTUGAMARKER_H
