/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *********************************************************************/

/**< \author Kevin Watts */
/**< \brief Recovers hardware ID from IMU device */


#include <string>
#include <boost/format.hpp>

#include "ros/console.h"
#include "microstrain_3dmgx2_imu/3dmgx2.h"
#include "log4cxx/logger.h"

std::string getID(microstrain_3dmgx2_imu::IMU &imu)
{
  char dev_name[17];
  char dev_model_num[17];
  char dev_serial_num[17];
  char dev_opt[17];
  imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_DEVICE_NAME, dev_name);
  imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_MODEL_NUMBER, dev_model_num);
  imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_SERIAL_NUMBER, dev_serial_num);
  imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_DEVICE_OPTIONS, dev_opt);
  
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

int main(int argc, char **argv)
{
  if (argc < 2 || argc > 3)
  {
    fprintf(stderr, "usage: get_id /dev/ttyUSB? [quiet]\nOutputs the device ID of an IMU at that port. Add a second argument for script friendly output.\n");
    return 1;
  }

  bool verbose = (argc == 2);
  if (!verbose)
  {
    // In quiet mode we want to turn off logging levels that go to stdout.
    log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Error]);
    ros::console::notifyLoggerLevelsChanged();
  }

  microstrain_3dmgx2_imu::IMU imu;

  try
  {
    imu.openPort(argv[1]);
  }
  catch (microstrain_3dmgx2_imu::Exception& e) 
  {
    fprintf(stderr, "Unable to open IMU at port %s. IMU may be disconnected.\n%s", argv[1], e.what());
    return 1;
  }

  imu.initTime(0.0);
  
  std::string id = getID(imu);

  if (verbose)
    fprintf(stdout, "IMU Device at port %s has ID: ", argv[1]);
  fprintf(stdout, "%s\n", id.c_str());

  try
  {
    imu.closePort();
  } 
  catch (microstrain_3dmgx2_imu::Exception& e) 
  {
    fprintf(stderr, "Exception thrown while stopping IMU.\n%s", e.what());
    return 1;
  }
  
  return 0;
}
