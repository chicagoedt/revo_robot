/* -*- mode: C++ -*- */
/* $Id: driver1394.h 35611 2011-01-30 05:49:18Z joq $ */

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010 Jack O'Quin
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <driver_base/driver.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include "dev_camera1394stereo.h"
#include "camera1394stereo/Camera1394StereoConfig.h"
typedef camera1394stereo::Camera1394StereoConfig Config;

/** @file

    @brief ROS driver interface for IIDC-compatible IEEE 1394 digital cameras.

*/

namespace camera1394stereo_driver
{

class Camera1394StereoDriver
{
public:

  // public methods
  Camera1394StereoDriver(ros::NodeHandle priv_nh,
                   ros::NodeHandle camera_nh);
  ~Camera1394StereoDriver();
  void poll(void);
  void setup(void);
  void shutdown(void);

private:

  enum CameraSelector {LEFT=0,RIGHT=1};
  static const int NUM_CAMERAS = 2;
  static const std::string CameraSelectorString[NUM_CAMERAS]; // = {"left","right"};

  // private methods
  void closeCamera();
  bool openCamera(Config &newconfig);
  void publish(const sensor_msgs::ImagePtr image[NUM_CAMERAS]);
  bool read(sensor_msgs::ImagePtr image[NUM_CAMERAS]);
  void reconfig(camera1394stereo::Camera1394StereoConfig &newconfig, uint32_t level);

  /** Non-recursive mutex for serializing callbacks with device polling. */
  boost::mutex mutex_;

  volatile driver_base::Driver::state_t state_; // current driver state
  volatile bool reconfiguring_;        // true when reconfig() running

  ros::NodeHandle priv_nh_;             // private node handle
  ros::NodeHandle camera_nh_;           // camera name space handle
  ros::NodeHandle single_camera_nh_[NUM_CAMERAS]; // left/right camera name space handle
  std::string camera_name_;             // camera name

  /** libdc1394 camera device interface */
  boost::shared_ptr<camera1394stereo::Camera1394Stereo> dev_;

  /** dynamic parameter configuration */
  camera1394stereo::Camera1394StereoConfig config_;
  dynamic_reconfigure::Server<camera1394stereo::Camera1394StereoConfig> srv_;
  ros::Rate cycle_;                     // polling rate when closed

  /** camera calibration information */
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_[NUM_CAMERAS];
  bool calibration_matches_[NUM_CAMERAS];            // CameraInfo matches video mode

  /** image transport interfaces */
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher image_pub_[NUM_CAMERAS];

}; // end class Camera1394Driver

}; // end namespace camera1394_driver
