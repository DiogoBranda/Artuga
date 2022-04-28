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

#include "artugaMarker.h"

/**
 * @brief arucoCras  - Class constructor
 * @param id - id of aruco (INT)(in)
 * @param size - size of aruco (float)(in)
 */
artugaMarker::artugaMarker(int id,float size)
{
  this->id = id;
  this->size = size;

}

/**
 * @brief setArucoPose - set aruco pos with the values post on the function
 * @param x - x cordenate value (float)(in)
 * @param y - y cordenate value (float)(in)
 * @param z - z cordenate value (float)(in)
 * @param state - state 1 if aruco is detect 0 if not (float)(in)
 * @param Q1 - Quaternion value (Eigen::Quaterniond)(in)
 */
void artugaMarker::setArucoPose(float x, float y, float z, bool state,const Eigen::Quaterniond &Q1){

  this->poseAruco.header.stamp=ros::Time();
  this->poseAruco.pose.position.x = x;
  this->poseAruco.pose.position.y = y;
  this->poseAruco.pose.position.z = z;
  tf::quaternionEigenToMsg(Q1, this->poseAruco.pose.orientation);
  this->state = state;
}

/**
 * @brief getArucoPose - provides the pose of aruco relative to something
 * @return returns the aruco pose (geometry_msgs::PoseStamped)
 */
geometry_msgs::PoseStamped artugaMarker::getArucoPose(){
  return this->poseAruco;
}

/**
 * @brief getId - provides the aruco id
 * @return aruco id (int)
 */
int artugaMarker::getId(){
  return this->id;
}

/**
 * @brief getSize - provides the aruco size
 * @return aruco size (float)
 */
float artugaMarker::getSize(){
  return this->size;
}

/**
 * @brief getState - provides the aruco state
 * @return aruco state, 0 if not detect 1 if is detect (float)
 */
bool artugaMarker::getState(){
  return this->state;
}

/**
 * @brief detector given an image it detects the arucos an estimate the position,
 * then stores on the variable aruco passe on the function.
 * @param img - image to try detect the arucos (cv::Mat)(in)
 * @param aruco - aruco variable where the positions of arucos that where detect will be stored (arucoCras **)(out)
 * @param n - number of arucos on that exist (int)(in)
 * @param cameraMatrix - camera matrix (cv::Mat)(in)
 * @param distCoeffs -  camera distorcion values (cv::Mat)(in)
 */
void artugaMarker::detector(cv::Mat img, artugaMarker **aruco, int n, cv::Mat cameraMatrix,cv::Mat distCoeffs, std::string display,bool flag){

  /*#####################
   *##    Variables    ##
   *#####################*/
  std::vector<cv::Vec3d> rvecs, tvecs;// rotation and translation matrix
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
  std::vector<int> markerIds;//Marker ID
  std::vector<std::vector<cv::Point2f>> corners;// Marker corners
  cv::Mat imageCopy;
  img.copyTo(imageCopy);

  /*######################
   *##    Main cicle    ##
   *######################*/

  cv::aruco::detectMarkers(img, dictionary, corners, markerIds);

  if (markerIds.size()>0){
    if(corners.size()>0){

      if(flag)
        cv::aruco::drawDetectedMarkers(imageCopy, corners, markerIds);

      bool foundId = false;// set to true when an aruco that existes on the plataform is detected on the image

      /**
       * check if any of the arucos found on the image are on the **aruco variable
       * */
      //Cicle for the **aruco variable
      for (int f=0;f<n;f++){
        foundId = false;// everytime a new cicle begins set foundId flag to false

        //cicle for the arucos detect on the image
        for(int i=0; i<markerIds.size(); i++){

          //if the aruco on the image is equal to an aruco on the **aruco variable then it as found a valid aruco
          if(markerIds[i]==aruco[f]->getId()){
            foundId = true;
            /**
             * @brief cv::aruco::estimatePoseSingleMarkers estimates the relative position to the aruco seen by the camera
             */
            cv::aruco::estimatePoseSingleMarkers(corners, aruco[f]->getSize(), cameraMatrix, distCoeffs, rvecs, tvecs);

            // Build identity rotation matrix as a cv::Mat
            cv::Mat rot(3, 3, CV_64FC1);
            cv::Rodrigues(rvecs.data()[i], rot);
            // Convert to a tf2::Matrix3x3
            Eigen::Matrix3d tf2_rot;
            tf2_rot <<rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2);

            //Create a quarternion from the rotation matrix
            Eigen::Quaterniond Q1(tf2_rot);

            //Set the pose of the **aruco variable to the value found on the aruco present in the image and set the state to 1
            aruco[f]->setArucoPose((float) tvecs.data()[i][0],(float) tvecs.data()[i][1],(float) tvecs.data()[i][2],true,Q1);
            break;

          }

        }
        // If not found set pos and state to 0
        if(!foundId){
          aruco[f]->setArucoPose((float)0,(float) 0,(float) 0,false, Eigen::Quaterniond());
        }

      }

    }
    else {
        //Zero if no pose can be calculated
      for (int f=0;f<n;f++)
        aruco[f]->setArucoPose((float)0,(float) 0,(float) 0, false,Eigen::Quaterniond());

    }

  }

  else {
     //Zero if no marker was detected
    for (int f=0;f<n;f++)
      aruco[f]->setArucoPose((float)0,(float) 0,(float) 0, 0,Eigen::Quaterniond());
  }
  if(flag){
    cv::resize(imageCopy,imageCopy,cv::Size(960,540));
    cv::imshow(display, imageCopy);
    char key = (char) cv::waitKey(1);
  }

}


