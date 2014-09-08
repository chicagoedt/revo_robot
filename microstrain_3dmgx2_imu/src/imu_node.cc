/*
 * Software License Agreement (BSD License)
 *
 *  Microstrain 3DM-GX2 node
 *  Copyright (c) 2008-2010, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 */

#include <assert.h>
#include <math.h>
#include <iostream>

#include <boost/format.hpp>

#include "microstrain_3dmgx2_imu/3dmgx2.h"

#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"

#include "tf/transform_datatypes.h"
#include "microstrain_3dmgx2_imu/AddOffset.h"

#include "std_msgs/Bool.h"

using namespace std;

class ImuNode 
{
public:
  microstrain_3dmgx2_imu::IMU imu;
  sensor_msgs::Imu reading;

  string port;

  microstrain_3dmgx2_imu::IMU::cmd cmd;

  self_test::TestRunner self_test_;
  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher imu_data_pub_;
  ros::ServiceServer add_offset_serv_;
  ros::ServiceServer calibrate_serv_;
  ros::Publisher is_calibrated_pub_;

  bool running;

  bool autocalibrate_;
  bool calibrate_requested_;
  bool calibrated_;
  
  int error_count_;
  int slow_count_;
  std::string was_slow_;
  std::string error_status_;

  string frameid_;
  
  double offset_;
    
  double bias_x_;
  double bias_y_;
  double bias_z_;

  double angular_velocity_stdev_, angular_velocity_covariance_;
  double linear_acceleration_covariance_, linear_acceleration_stdev_;
  double orientation_covariance_, orientation_stdev_;

  double max_drift_rate_;

  double desired_freq_;
  diagnostic_updater::FrequencyStatus freq_diag_;
  
  ImuNode(ros::NodeHandle h) : self_test_(), diagnostic_(), 
  node_handle_(h), private_node_handle_("~"), calibrate_requested_(false),
  error_count_(0), 
  slow_count_(0), 
  desired_freq_(100), 
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
  {
    ros::NodeHandle imu_node_handle(node_handle_, "imu");
    
    private_node_handle_.param("autocalibrate", autocalibrate_, true);
    private_node_handle_.param("assume_calibrated", calibrated_, false);
    private_node_handle_.param("port", port, string("/dev/ttyUSB0"));
    private_node_handle_.param("max_drift_rate", max_drift_rate_, 0.0002);


    imu_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data", 100);
    add_offset_serv_ = private_node_handle_.advertiseService("add_offset", &ImuNode::addOffset, this);
    calibrate_serv_ = imu_node_handle.advertiseService("calibrate", &ImuNode::calibrate, this);
    is_calibrated_pub_ = imu_node_handle.advertise<std_msgs::Bool>("is_calibrated", 1, true);

    publish_is_calibrated();

    cmd = microstrain_3dmgx2_imu::IMU::CMD_ACCEL_ANGRATE_ORIENT;
    
    running = false;

    bias_x_ = bias_y_ = bias_z_ = 0;

    private_node_handle_.param("frame_id", frameid_, string("imu"));
    reading.header.frame_id = frameid_;

    private_node_handle_.param("time_offset", offset_, 0.0);

    private_node_handle_.param("linear_acceleration_stdev", linear_acceleration_stdev_, 0.098); 
    private_node_handle_.param("orientation_stdev", orientation_stdev_, 0.035); 
    private_node_handle_.param("angular_velocity_stdev", angular_velocity_stdev_, 0.012); 

    double angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    double orientation_covariance = orientation_stdev_ * orientation_stdev_;
    double linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    
    reading.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    reading.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    reading.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    reading.angular_velocity_covariance[0] = angular_velocity_covariance;
    reading.angular_velocity_covariance[4] = angular_velocity_covariance;
    reading.angular_velocity_covariance[8] = angular_velocity_covariance;
    
    reading.orientation_covariance[0] = orientation_covariance;
    reading.orientation_covariance[4] = orientation_covariance;
    reading.orientation_covariance[8] = orientation_covariance;
    
    self_test_.add("Close Test", this, &ImuNode::pretest);
    self_test_.add("Interruption Test", this, &ImuNode::InterruptionTest);
    self_test_.add("Connect Test", this, &ImuNode::ConnectTest);
    self_test_.add("Read ID Test", this, &ImuNode::ReadIDTest);
    self_test_.add("Gyro Bias Test", this, &ImuNode::GyroBiasTest);
    self_test_.add("Streamed Data Test", this, &ImuNode::StreamedDataTest);
    self_test_.add("Gravity Test", this, &ImuNode::GravityTest);
    self_test_.add("Disconnect Test", this, &ImuNode::DisconnectTest);
    self_test_.add("Resume Test", this, &ImuNode::ResumeTest);

    diagnostic_.add( freq_diag_ );
    diagnostic_.add( "Calibration Status", this, &ImuNode::calibrationStatus );
    diagnostic_.add( "IMU Status", this, &ImuNode::deviceStatus );
  }

  ~ImuNode()
  {
    stop();
  }

  void setErrorStatusf(const char *format, ...)
  {
    va_list va;
    char buff[1000]; 
    va_start(va, format);
    if (vsnprintf(buff, 1000, format, va) >= 1000)
      ROS_DEBUG("Really long string in setErrorStatus, it was truncated.");
    std::string value = std::string(buff);
    setErrorStatus(buff);
  }

  // Prints an error message if it isn't the same old error message.
  void setErrorStatus(const std::string msg)
  {
    if (error_status_ != msg)
    {
      error_status_ = msg;
      ROS_ERROR("%s You may find further details at http://www.ros.org/wiki/microstrain_3dmgx2_imu/Troubleshooting", msg.c_str());
    }
    else
    {
      ROS_DEBUG("%s You may find further details at http://www.ros.org/wiki/microstrain_3dmgx2_imu/Troubleshooting", msg.c_str());
    }
  } 

  void clearErrorStatus()
  {
    error_status_.clear();
  }

  int start()
  {
    stop();

    try
    {
      try
      {
        imu.openPort(port.c_str());
      } catch (microstrain_3dmgx2_imu::Exception& e) {
        error_count_++;
        setErrorStatus(e.what());
        diagnostic_.broadcast(2, e.what());
        return -1;
      }

      diagnostic_.setHardwareID(getID(true));

      ROS_INFO("Initializing IMU time with offset %f.", offset_);

      imu.initTime(offset_);

      if (autocalibrate_ || calibrate_requested_)
      {
        doCalibrate();
        calibrate_requested_ = false;
        autocalibrate_ = false; // No need to do this each time we reopen the device.
      }
      else
      {
        ROS_INFO("Not calibrating the IMU sensor. Use the calibrate service to calibrate it before use.");
      }

      ROS_INFO("IMU sensor initialized.");

      imu.setContinuous(cmd);

      freq_diag_.clear();

      running = true;

    } catch (microstrain_3dmgx2_imu::Exception& e) {
      error_count_++;
      usleep(100000); // Give isShuttingDown a chance to go true.
      if (!ros::isShuttingDown()){ // Don't warn if we are shutting down.
        setErrorStatusf("Exception thrown while starting IMU. This sometimes happens if you are not connected to an IMU or if another process is trying to access the IMU port. You may try 'lsof|grep %s' to see if other processes have the port open. %s", port.c_str(), e.what());
        diagnostic_.broadcast(2, "Error opening IMU.");
      }
      return -1;
    }

    return(0);
  }
      
  std::string getID(bool output_info = false)
  {
      char dev_name[17];
      char dev_model_num[17];
      char dev_serial_num[17];
      char dev_opt[17];
      imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_DEVICE_NAME, dev_name);
      imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_MODEL_NUMBER, dev_model_num);
      imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_SERIAL_NUMBER, dev_serial_num);
      imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_DEVICE_OPTIONS, dev_opt);
      
      if (output_info)
        ROS_INFO("Connected to IMU [%s] model [%s] s/n [%s] options [%s]",
          dev_name, dev_model_num, dev_serial_num, dev_opt);
      
      char *dev_name_ptr = dev_name;
      char *dev_model_num_ptr = dev_model_num;
      char *dev_serial_num_ptr = dev_serial_num;
      
      while (*dev_name_ptr == ' ')
        dev_name_ptr++;
      while (*dev_model_num_ptr == ' ')
        dev_model_num_ptr++;
      while (*dev_serial_num_ptr == ' ')
        dev_serial_num_ptr++;

      return (boost::format("%s_%s-%s")%dev_name_ptr%dev_model_num_ptr%dev_serial_num_ptr).str();
  }
  
  int stop()
  {
    if(running)
    {
      try
      {
        imu.closePort();
      } catch (microstrain_3dmgx2_imu::Exception& e) {
        error_count_++;
        ROS_INFO("Exception thrown while stopping IMU. %s", e.what());
      }
      running = false;
    }

    return(0);
  }

  int publish_datum()
  {
    try
    {
      static double prevtime = 0;
      double starttime = ros::Time::now().toSec();
      if (prevtime && prevtime - starttime > 0.05)
      {
        ROS_WARN("Full IMU loop took %f ms. Nominal is 10ms.", 1000 * (prevtime - starttime));
        was_slow_ = "Full IMU loop was slow.";
        slow_count_++;
      }
      getData(reading);
      double endtime = ros::Time::now().toSec();
      if (endtime - starttime > 0.05)
      {
        ROS_WARN("Gathering data took %f ms. Nominal is 10ms.", 1000 * (endtime - starttime));
        was_slow_ = "Full IMU loop was slow.";
        slow_count_++;
      }
      prevtime = starttime;
      starttime = ros::Time::now().toSec();
      imu_data_pub_.publish(reading);
      endtime = ros::Time::now().toSec();
      if (endtime - starttime > 0.05)
      {
        ROS_WARN("Publishing took %f ms. Nominal is 10 ms.", 1000 * (endtime - starttime));
        was_slow_ = "Full IMU loop was slow.";
        slow_count_++;
      }
        
      freq_diag_.tick();
      clearErrorStatus(); // If we got here, then the IMU really is working. Next time an error occurs, we want to print it.
    } catch (microstrain_3dmgx2_imu::Exception& e) {
      error_count_++;
      usleep(100000); // Give isShuttingDown a chance to go true.
      if (!ros::isShuttingDown()) // Don't warn if we are shutting down.
        ROS_WARN("Exception thrown while trying to get the IMU reading. This sometimes happens due to a communication glitch, or if another process is trying to access the IMU port. You may try 'lsof|grep %s' to see if other processes have the port open. %s", port.c_str(), e.what());
      return -1;
    }

    return(0);
  }

  bool spin()
  {
    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
      if (start() == 0)
      {
        while(node_handle_.ok()) {
          if(publish_datum() < 0)
            break;
          self_test_.checkTest();
          diagnostic_.update();
          ros::spinOnce();
        }
      } else {
        // No need for diagnostic here since a broadcast occurs in start
        // when there is an error.
        usleep(1000000);
        self_test_.checkTest();
        ros::spinOnce();
      }
    }

    stop();

    return true;
  }

  void publish_is_calibrated()
  {
    std_msgs::Bool msg;
    msg.data = calibrated_;
    is_calibrated_pub_.publish(msg);
  }

  void pretest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    try
    {
      imu.closePort();

      status.summary(0, "Device closed successfully.");
    } catch (microstrain_3dmgx2_imu::Exception& e) {
      status.summary(1, "Failed to close device.");
    }
  }

  void InterruptionTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (imu_data_pub_.getNumSubscribers() == 0 )
      status.summary(0, "No operation interrupted.");
    else
      status.summary(1, "There were active subscribers.  Running of self test interrupted operations.");
  }

  void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    imu.openPort(port.c_str());

    status.summary(0, "Connected successfully.");
  }

  void ReadIDTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    self_test_.setID(getID());
    
    status.summary(0, "Read Successfully");
  }

  void GyroBiasTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    imu.initGyros(&bias_x_, &bias_y_, &bias_z_);

    status.summary(0, "Successfully calculated gyro biases.");

    status.add("Bias_X", bias_x_);
    status.add("Bias_Y", bias_y_);
    status.add("Bias_Z", bias_z_);
  }


  void getData(sensor_msgs::Imu& data)
  {
    uint64_t time;
    double accel[3];
    double angrate[3];
    double orientation[9];

    imu.receiveAccelAngrateOrientation(&time, accel, angrate, orientation);
    data.linear_acceleration.x = accel[0];
    data.linear_acceleration.y = accel[1];
    data.linear_acceleration.z = accel[2];
 
    data.angular_velocity.x = angrate[0];
    data.angular_velocity.y = angrate[1];
    data.angular_velocity.z = angrate[2];
      
    tf::Quaternion quat;
    (tf::Matrix3x3(-1,0,0,
		 0,1,0,
		 0,0,-1)*
    tf::Matrix3x3(orientation[0], orientation[3], orientation[6],
		 orientation[1], orientation[4], orientation[7],
		 orientation[2], orientation[5], orientation[8])).getRotation(quat);
    
    tf::quaternionTFToMsg(quat, data.orientation);
      
    data.header.stamp = ros::Time::now().fromNSec(time);
  }


  void StreamedDataTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    uint64_t time;
    double accel[3];
    double angrate[3];

    if (!imu.setContinuous(microstrain_3dmgx2_imu::IMU::CMD_ACCEL_ANGRATE))
    {
      status.summary(2, "Could not start streaming data.");
    } else {

      for (int i = 0; i < 100; i++)
      {
        imu.receiveAccelAngrate(&time, accel, angrate);
      }
      
      imu.stopContinuous();

      status.summary(0, "Data streamed successfully.");
    }
  }

  void GravityTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    uint64_t time;
    double accel[3];
    double angrate[3];

    double grav = 0.0;

    double grav_x = 0.0;
    double grav_y = 0.0;
    double grav_z = 0.0;

    if (!imu.setContinuous(microstrain_3dmgx2_imu::IMU::CMD_ACCEL_ANGRATE))
    {
      status.summary(2, "Could not start streaming data.");
    } else {

      int num = 200;

      for (int i = 0; i < num; i++)
      {
        imu.receiveAccelAngrate(&time, accel, angrate);
        
        grav_x += accel[0];
        grav_y += accel[1];
        grav_z += accel[2];

      }
      
      imu.stopContinuous();

      grav += sqrt( pow(grav_x / (double)(num), 2.0) + 
                    pow(grav_y / (double)(num), 2.0) + 
                    pow(grav_z / (double)(num), 2.0));
      
      //      double err = (grav - microstrain_3dmgx2_imu::G);
      double err = (grav - 9.796);
      
      if (fabs(err) < .05)
      {
        status.summary(0, "Gravity detected correctly.");
      } else {
        status.summaryf(2, "Measured gravity deviates by %f", err);
      }

      status.add("Measured gravity", grav);
      status.add("Gravity error", err);
    }
  }

  void DisconnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    imu.closePort();

    status.summary(0, "Disconnected successfully.");
  }

  void ResumeTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (running)
    {

      imu.openPort(port.c_str());
      freq_diag_.clear();

      if (imu.setContinuous(cmd) != true)
      {
        status.summary(2, "Failed to resume previous mode of operation.");
        return;
      }
    }

    status.summary(0, "Previous operation resumed successfully.");    
  }

  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if (!running)
      status.summary(2, "IMU is stopped");
    else if (!was_slow_.empty())
    {
      status.summary(1, "Excessive delay");
      was_slow_.clear();
    }
    else
      status.summary(0, "IMU is running");

    status.add("Device", port);
    status.add("TF frame", frameid_);
    status.add("Error count", error_count_);
    status.add("Excessive delay", slow_count_);
  }

  void calibrationStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (calibrated_)
    {
      status.summary(0, "Gyro is calibrated");
      status.add("X bias", bias_x_);
      status.add("Y bias", bias_y_);
      status.add("Z bias", bias_z_);
    }
    else
      status.summary(2, "Gyro not calibrated");
      // Note to Kevin. Yes the IMU is running, but it is outputting
      // garbage, so this is an error.

  }

  bool addOffset(microstrain_3dmgx2_imu::AddOffset::Request &req, microstrain_3dmgx2_imu::AddOffset::Response &resp)
  {
    double offset = req.add_offset;
    offset_ += offset;

    ROS_INFO("Adding %f to existing IMU time offset.", offset);
    ROS_INFO("Total IMU time offset is now %f.", offset_);

    // send changes to imu driver
    imu.setFixedOffset(offset_);

    // write changes to param server
    private_node_handle_.setParam("time_offset", offset_);

    // set response
    resp.total_offset = offset_;

    return true;
  }

  bool calibrate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    bool old_running = running;

    try
    {
      calibrate_requested_ = true;
      if (old_running)
      {
        stop();
        start(); // Start will do the calibration.
      }
      else
      {
        imu.openPort(port.c_str());
        doCalibrate();
        imu.closePort();
      } 
    } catch (microstrain_3dmgx2_imu::Exception& e) {
      error_count_++;
      calibrated_ = false;
      publish_is_calibrated();
      ROS_ERROR("Exception thrown while calibrating IMU %s", e.what());
      stop();
      if (old_running)
        start(); // Might throw, but we have nothing to lose... Needs restructuring.
      return false;
    }
    
    return true;
  }

  void doCalibrate()
  { // Expects to be called with the IMU stopped.
    ROS_INFO("Calibrating IMU gyros.");
    imu.initGyros(&bias_x_, &bias_y_, &bias_z_);
    
    // check calibration
    if (!imu.setContinuous(microstrain_3dmgx2_imu::IMU::CMD_ACCEL_ANGRATE_ORIENT)){
      ROS_ERROR("Could not start streaming data to verify calibration");
    } 
    else {
      double x_rate = 0.0;
      double y_rate = 0.0;
      double z_rate = 0.0;
      size_t count = 0;
      getData(reading);
      ros::Time start_time = reading.header.stamp;

      while(reading.header.stamp - start_time < ros::Duration(2.0)){
        getData(reading);
        x_rate += reading.angular_velocity.x;
        y_rate += reading.angular_velocity.y;
        z_rate += reading.angular_velocity.z;
        ++count;
      }

      double average_rate = sqrt(x_rate*x_rate + y_rate*y_rate + z_rate*z_rate) / count;

      if (count < 200){
        ROS_WARN("Imu: calibration check acquired fewer than 200 samples.");
      }
      
      // calibration succeeded
      if (average_rate < max_drift_rate_) {
        ROS_INFO("Imu: calibration check succeeded: average angular drift %f mdeg/sec < %f mdeg/sec", average_rate*180*1000/M_PI, max_drift_rate_*180*1000/M_PI);
        calibrated_ = true;
        publish_is_calibrated();
        ROS_INFO("IMU gyro calibration completed.");
        freq_diag_.clear();
      }
      // calibration failed
      else{
        calibrated_ = false;
        publish_is_calibrated();
        ROS_ERROR("Imu: calibration check failed: average angular drift = %f mdeg/sec > %f mdeg/sec", average_rate*180*1000/M_PI, max_drift_rate_*180*1000/M_PI);
      }
      imu.stopContinuous();
    }
  }
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "microstrain_3dmgx2_node");

  ros::NodeHandle nh;

  ImuNode in(nh);
  in.spin();

  return(0);
}
