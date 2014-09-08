// $Id: driver1394.cpp 35691 2011-02-02 04:28:58Z joq $

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2009, 2010 Jack O'Quin, Patrick Beeson
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

#include <boost/format.hpp>

#include <driver_base/SensorLevels.h>
#include <tf/transform_listener.h>

#include "driver1394stereo.h"
#include "camera1394stereo/Camera1394StereoConfig.h"
#include "featuresstereo.h"

/** @file

@brief ROS driver for IIDC-compatible IEEE 1394 digital cameras.

This is a ROS driver for 1394 cameras, using libdc1394.  It can be
instantiated as either a node or a nodelet.  It is written with with
minimal dependencies, intended to fill a role in the ROS image
pipeline similar to the other ROS camera drivers.

@par Advertises

 - @b stereo_camera/left/image_raw topic (sensor_msgs/Image) raw 2D camera images

 - @b stereo_camera/right/image_raw topic (sensor_msgs/Image) raw 2D camera images

 - @b stereo_camera/left/camera_info topic (sensor_msgs/CameraInfo) Calibration 
      information for each image.

 - @b stereo_camera/right/camera_info topic (sensor_msgs/CameraInfo) Calibration    
      information for each image.



*/

namespace camera1394stereo_driver
{
  // some convenience typedefs
  typedef camera1394stereo::Camera1394StereoConfig Config;
  typedef driver_base::Driver Driver;
  typedef driver_base::SensorLevels Levels;

  const std::string Camera1394StereoDriver::CameraSelectorString[NUM_CAMERAS] = {"left","right"};

  Camera1394StereoDriver::Camera1394StereoDriver(ros::NodeHandle priv_nh,
                                                 ros::NodeHandle camera_nh):
    state_(Driver::CLOSED),
    reconfiguring_(false),
    priv_nh_(priv_nh),
    camera_nh_(camera_nh),
    camera_name_("stereo_camera"),
    dev_(new camera1394stereo::Camera1394Stereo()),
    srv_(priv_nh),
    cycle_(1.0),                        // slow poll when closed
    it_(new image_transport::ImageTransport(camera_nh_))
  {
    for (int i=0; i< NUM_CAMERAS; i++)
    {
      single_camera_nh_[i] = ros::NodeHandle(camera_nh_,CameraSelectorString[i]);  // for i-th CameraInfoManager
      cinfo_[i] = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(single_camera_nh_[i]));
      calibration_matches_[i] = true;
      image_pub_[i] = it_->advertiseCamera(CameraSelectorString[i]+"/image_raw", 1);
    }
  }

  Camera1394StereoDriver::~Camera1394StereoDriver()
  {}

  /** Close camera device
   *
   *  postcondition: state_ is Driver::CLOSED
   */
  void Camera1394StereoDriver::closeCamera()
  {
    if (state_ != Driver::CLOSED)
      {
        ROS_INFO_STREAM("[" << camera_name_ << "] closing device");
        dev_->close();
        state_ = Driver::CLOSED;
      }
  }

  /** Open the camera device.
   *
   * @param newconfig configuration parameters
   * @return true, if successful
   *
   * if successful:
   *   state_ is Driver::OPENED
   *   camera_name_ set to GUID string
   */
  bool Camera1394StereoDriver::openCamera(Config &newconfig)
  {
    bool success = false;
    int retries = 2;                    // number of retries, if open fails
    do
      {
        try
          {
            if (0 == dev_->open(newconfig))
              {
                if (camera_name_ != dev_->device_id_)
                  {
                    camera_name_ = dev_->device_id_;
                    for (int i=0; i<NUM_CAMERAS; i++)
                      if ( ! cinfo_[i]->setCameraName(camera_name_ + "_" +
                                                      CameraSelectorString[i] ) )
                        {
                          // GUID is 16 hex digits, which should be valid.
                          // If not, use it for log messages anyway.
                          ROS_WARN_STREAM("[" << camera_name_
                                          <<"_"<<CameraSelectorString[i]
                                          << "] name not valid"
                                          << " for camera_info_manger");
                        }
                  }
                ROS_INFO_STREAM("[" << camera_name_ << "] opened: "
                                << newconfig.video_mode << ", "
                                << newconfig.frame_rate << " fps, "
                                << newconfig.iso_speed << " Mb/s");
                state_ = Driver::OPENED;
                for (int i=0; i<NUM_CAMERAS; i++)
                  calibration_matches_[i] = true;
                success = true;
              }
          }
        catch (camera1394stereo::Exception& e)
          {
            state_ = Driver::CLOSED;    // since the open() failed
            if (retries > 0)
              {
                ROS_WARN_STREAM("[" << camera_name_
                                << "] exception opening device (retrying): "
                                << e.what());
                ROS_WARN_STREAM("Trying to reset bus with system call...");
                if ( system("dc1394_reset_bus") == 0 )
                  ROS_WARN_STREAM("Bus reset system call successful");
                else
                  ROS_WARN_STREAM("Bus reset system call failure");
              }
            else
              ROS_ERROR_STREAM("[" << camera_name_
                               << "] device open failed: " << e.what());
          }
      }
    while (!success && --retries >= 0);

    return success;
  }


  /** device poll */
  void Camera1394StereoDriver::poll(void)
  {
    // Do not run concurrently with reconfig().
    //
    // The mutex lock should be sufficient, but the Linux pthreads
    // implementation does not guarantee fairness, and the reconfig()
    // callback thread generally suffers from lock starvation for many
    // seconds before getting to run.  So, we avoid acquiring the lock
    // if there is a reconfig() pending.
    bool do_sleep = true;
    if (!reconfiguring_)
      {
        boost::mutex::scoped_lock lock(mutex_);
        do_sleep = (state_ == Driver::CLOSED);
        if (!do_sleep)
          {
            // driver is open, read the next image still holding lock
            sensor_msgs::ImagePtr image[NUM_CAMERAS];
            for (int i=0; i<NUM_CAMERAS; i++)
              image[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);
            if (read(image))
              {
                publish(image);
              }
          }
      }

    if (do_sleep)
      {
        // device was closed or poll is not running, sleeping avoids
        // busy wait (DO NOT hold the lock while sleeping)
        cycle_.sleep();
      }
  }

  /** Publish camera stream topics
   *
   *  @param image points to latest camera frame
   */
  void Camera1394StereoDriver::publish(const sensor_msgs::ImagePtr image[NUM_CAMERAS])
  {
    for (int i=0; i<NUM_CAMERAS; i++)
      {
        image[i]->header.frame_id = config_.frame_id;

        // get current CameraInfo data
        sensor_msgs::CameraInfoPtr
          ci(new sensor_msgs::CameraInfo(cinfo_[i]->getCameraInfo()));

        // check whether CameraInfo matches current video mode
        if (!dev_->checkCameraInfo(*(image[i]), *ci))
          {
            // image size does not match: publish a matching uncalibrated
            // CameraInfo instead
            if (calibration_matches_[i])
              {
                // warn user once
                calibration_matches_[i] = false;
                ROS_WARN_STREAM("[" << camera_name_
                                << "_" << CameraSelectorString[i]
                                << "] calibration does not match video mode "
                                << "(publishing uncalibrated data)");
              }
            ci.reset(new sensor_msgs::CameraInfo());
            ci->height = image[i]->height;
            ci->width = image[i]->width;
          }
        else if (!calibration_matches_[i])
          {
            // calibration OK now
            calibration_matches_[i] = true;
            ROS_WARN_STREAM("[" << camera_name_
                            << "_" << CameraSelectorString[i]
                            << "] calibration matches video mode now");
          }

        // fill in operational parameters
        dev_->setOperationalParameters(*ci);

        ci->header.frame_id = config_.frame_id;
        ci->header.stamp = image[i]->header.stamp;

        // @todo log a warning if (filtered) time since last published
        // image is not reasonably close to configured frame_rate

        // Publish via image_transport
        image_pub_[i].publish(image[i], ci);
      }
  }

  /** Read camera data.
   *
   * @param image points to camera Image message
   * @return true if successful, with image filled in
   */
  bool Camera1394StereoDriver::read(sensor_msgs::ImagePtr image[NUM_CAMERAS])
  {
    bool success = true;
    try
      {
        // Read data from the Camera
        ROS_DEBUG_STREAM("[" << camera_name_ << "] reading data");
        success = dev_->readData(*(image[LEFT]), *(image[RIGHT]));
        ROS_DEBUG_STREAM("[" << camera_name_ << "] read returned");
      }
    catch (camera1394stereo::Exception& e)
      {
        ROS_WARN_STREAM("[" << camera_name_
                        << "] Exception reading data: " << e.what());
        success = false;
      }
    return success;
  }

  /** Dynamic reconfigure callback
   *
   *  Called immediately when callback first defined. Called again
   *  when dynamic reconfigure starts or changes a parameter value.
   *
   *  @param newconfig new Config values
   *  @param level bit-wise OR of reconfiguration levels for all
   *               changed parameters (0xffffffff on initial call)
   **/
  void Camera1394StereoDriver::reconfig(Config &newconfig, uint32_t level)
  {
    // Do not run concurrently with poll().  Tell it to stop running,
    // and wait on the lock until it does.
    reconfiguring_ = true;
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG("dynamic reconfigure level 0x%x", level);

    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "")
      newconfig.frame_id = "camera";
    std::string tf_prefix = tf::getPrefixParam(priv_nh_);
    ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
    newconfig.frame_id = tf::resolve(tf_prefix, newconfig.frame_id);

    if (state_ != Driver::CLOSED && (level & Levels::RECONFIGURE_CLOSE))
      {
        // must close the device before updating these parameters
        closeCamera();                  // state_ --> CLOSED
      }

    if (state_ == Driver::CLOSED)
      {
        // open with new values
        if (openCamera(newconfig))
          {
            // update GUID string parameter
            // TODO move into dev_->open()
            newconfig.guid = camera_name_;
          }
      }

    std::string camera_info_url[NUM_CAMERAS];
    camera_info_url[LEFT] = config_.camera_info_url_left;
    camera_info_url[RIGHT] = config_.camera_info_url_right;
    std::string new_camera_info_url[NUM_CAMERAS];
    new_camera_info_url[LEFT] = newconfig.camera_info_url_left;
    new_camera_info_url[RIGHT] = newconfig.camera_info_url_right;

    for (int i=0; i<NUM_CAMERAS; i++)
      {
        if (camera_info_url[i] != new_camera_info_url[i])
          {
            // set the new URL and load CameraInfo (if any) from it
            if (cinfo_[i]->validateURL(new_camera_info_url[i]))
              {
                cinfo_[i]->loadCameraInfo(new_camera_info_url[i]);
              }
            else
              {
                // new URL not valid, use the old one
                ROS_WARN_STREAM("[" << camera_name_
                                    << "_" << CameraSelectorString[i]
                                    << "] not updating calibration to invalid URL "
                                    << new_camera_info_url[i] );
              }
          }
       }
    if (state_ != Driver::CLOSED)       // openCamera() succeeded?
      {
        // configure IIDC features
        if (level & Levels::RECONFIGURE_CLOSE)
          {
            // initialize all features for newly opened device
            if (false == dev_->features_->initialize(&newconfig))
              {
                ROS_ERROR_STREAM("[" << camera_name_
                                 << "] feature initialization failure");
                closeCamera();          // can't continue
              }
          }
        else
          {
            // update any features that changed
            // TODO replace this with a dev_->reconfigure(&newconfig);
            dev_->features_->reconfigure(&newconfig);
          }
      }

    config_ = newconfig;                // save new parameters
    reconfiguring_ = false;             // let poll() run again

    ROS_DEBUG_STREAM("[" << camera_name_
                     << "] reconfigured: frame_id " << newconfig.frame_id
                     << ", camera_info_url_left " << newconfig.camera_info_url_left
                     << ", camera_info_url_right " << newconfig.camera_info_url_right);
  }


  /** driver initialization
   *
   *  Define dynamic reconfigure callback, which gets called
   *  immediately with level 0xffffffff.  The reconfig() method will
   *  set initial parameter values, then open the device if it can.
   */
  void Camera1394StereoDriver::setup(void)
  {
    srv_.setCallback(boost::bind(&Camera1394StereoDriver::reconfig, this, _1, _2));
  }


  /** driver termination */
  void Camera1394StereoDriver::shutdown(void)
  {
    closeCamera();
  }

}; // end namespace camera1394_driver
