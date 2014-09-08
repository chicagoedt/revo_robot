/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-20010  Willow Garage
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <netinet/in.h>
#include <stdlib.h>

#include <sys/time.h>

//#include <ros/console.h>

#include "microstrain_3dmgx2_imu/3dmgx2.h"

#include "poll.h"


//! Macro for throwing an exception with a message
#define IMU_EXCEPT(except, msg, ...) \
  { \
    char buf[1000]; \
    snprintf(buf, 1000, msg" (in microstrain_3dmgx2_imu::IMU:%s)", ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
  }

// Some systems (e.g., OS X) require explicit externing of static class
// members.
extern const double microstrain_3dmgx2_imu::IMU::G;
extern const double microstrain_3dmgx2_imu::IMU::KF_K_1;
extern const double microstrain_3dmgx2_imu::IMU::KF_K_2;

//! Code to swap bytes since IMU is big endian
static inline unsigned short bswap_16(unsigned short x) {
  return (x>>8) | (x<<8);
}

//! Code to swap bytes since IMU is big endian
static inline unsigned int bswap_32(unsigned int x) {
  return (bswap_16(x&0xffff)<<16) | (bswap_16(x>>16));
}


//! Code to extract a floating point number from the IMU
static float extract_float(uint8_t* addr) {

  float tmp;

  *((unsigned char*)(&tmp) + 3) = *(addr);
  *((unsigned char*)(&tmp) + 2) = *(addr+1);
  *((unsigned char*)(&tmp) + 1) = *(addr+2);
  *((unsigned char*)(&tmp)) = *(addr+3);

  return tmp;
}


//! Helper function to get system time in nanoseconds.
static unsigned long long time_helper()
{
#if POSIX_TIMERS > 0
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  return (unsigned long long)(curtime.tv_sec) * 1000000000 + (unsigned long long)(curtime.tv_nsec);  
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  return (unsigned long long)(timeofday.tv_sec) * 1000000000 + (unsigned long long)(timeofday.tv_usec) * 1000;  
#endif
}


////////////////////////////////////////////////////////////////////////////////
// Constructor
microstrain_3dmgx2_imu::IMU::IMU() : fd(-1), continuous(false), is_gx3(false)
{ }


////////////////////////////////////////////////////////////////////////////////
// Destructor
microstrain_3dmgx2_imu::IMU::~IMU()
{
  closePort();
}


////////////////////////////////////////////////////////////////////////////////
// Open the IMU port
void
microstrain_3dmgx2_imu::IMU::openPort(const char *port_name)
{
  closePort(); // In case it was previously open, try to close it first.

  // Open the port
  fd = open(port_name, O_RDWR | O_SYNC | O_NONBLOCK | O_NOCTTY, S_IRUSR | S_IWUSR );
  if (fd < 0)
  {
    const char *extra_msg = "";
    switch (errno)
    {
      case EACCES:
        extra_msg = "You probably don't have premission to open the port for reading and writing.";
        break;
      case ENOENT:
        extra_msg = "The requested port does not exist. Is the IMU connected? Was the port name misspelled?";
        break;
    }

    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to open serial port [%s]. %s. %s", port_name, strerror(errno), extra_msg);
  }

  // Lock the port
  struct flock fl;
  fl.l_type   = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len   = 0;
  fl.l_pid   = getpid();

  if (fcntl(fd, F_SETLK, &fl) != 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);

  // Change port settings
  struct termios term;
  if (tcgetattr(fd, &term) < 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to get serial port attributes. The port you specified (%s) may not be a serial port.", port_name);

  cfmakeraw( &term );
  cfsetispeed(&term, B115200);
  cfsetospeed(&term, B115200);

  if (tcsetattr(fd, TCSAFLUSH, &term) < 0 )
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.

  // Stop continuous mode
  stopContinuous();

  // Make sure queues are empty before we begin
  if (tcflush(fd, TCIOFLUSH) != 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Tcflush failed. Please report this error if you see it.");
}


////////////////////////////////////////////////////////////////////////////////
// Close the IMU port
void
microstrain_3dmgx2_imu::IMU::closePort()
{
  if (fd != -1)
  {
    if (continuous)
    {
      try {
        //ROS_DEBUG("stopping continuous");
        stopContinuous();

      } catch (microstrain_3dmgx2_imu::Exception &e) {
        // Exceptions here are fine since we are closing anyways
      }
    }

    if (close(fd) != 0)
      IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Unable to close serial port; [%s]", strerror(errno));
    fd = -1;
  }
}



////////////////////////////////////////////////////////////////////////////////
// Initialize time information
void
microstrain_3dmgx2_imu::IMU::initTime(double fix_off)
{
  wraps = 0;

  uint8_t cmd[1];
  uint8_t rep[31];
  cmd[0] = CMD_RAW;

  transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);
  start_time = time_helper();

  int k = 25;
  offset_ticks = bswap_32(*(uint32_t*)(rep + k));
  last_ticks = offset_ticks;

  // reset kalman filter state
  offset = 0;
  d_offset = 0;
  sum_meas = 0;
  counter = 0;

  // fixed offset
  fixed_offset = fix_off;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize IMU gyros
void
microstrain_3dmgx2_imu::IMU::initGyros(double* bias_x, double* bias_y, double* bias_z)
{
  wraps = 0;

  uint8_t cmd[5];
  uint8_t rep[19];

  cmd[0] = CMD_CAPTURE_GYRO_BIAS;
  cmd[1] = 0xC1;
  cmd[2] = 0x29;
  *(unsigned short*)(&cmd[3]) = bswap_16(10000);

  transact(cmd, sizeof(cmd), rep, sizeof(rep), 30000);

  if (bias_x)
    *bias_x = extract_float(rep + 1);
  
  if (bias_y)
    *bias_y = extract_float(rep + 5);

  if (bias_z)
    *bias_z = extract_float(rep + 9);
}


////////////////////////////////////////////////////////////////////////////////
// Put the IMU into continuous mode
bool
microstrain_3dmgx2_imu::IMU::setContinuous(cmd command)
{
  uint8_t cmd[4];
  uint8_t rep[8];

  cmd[0] = CMD_CONTINUOUS;
  cmd[1] = 0xC1; //Confirms user intent
  cmd[2] = 0x29; //Confirms user intent
  cmd[3] = command;

  transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);
  
  // Verify that continuous mode is set on correct command:
  if (rep[1] != command) {
    return false;
  }

  continuous = true;
  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Take the IMU out of continuous mode
void
microstrain_3dmgx2_imu::IMU::stopContinuous()
{
  uint8_t cmd[3];

  cmd[0] = CMD_STOP_CONTINUOUS;
  
  cmd[1] = 0x75; // gx3 - confirms user intent

  cmd[2] = 0xb4; // gx3 - confirms user intent

  send(cmd, sizeof(cmd));

  send(cmd, is_gx3 ? 3 : 1);

  usleep(1000000);

  if (tcflush(fd, TCIOFLUSH) != 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "Tcflush failed");

  continuous = false;
}



////////////////////////////////////////////////////////////////////////////////
// Receive ACCEL_ANGRATE_MAG message
void
microstrain_3dmgx2_imu::IMU::receiveAccelAngrateMag(uint64_t *time, double accel[3], double angrate[3], double mag[3])
{
  int i, k;
  uint8_t rep[43];

  uint64_t sys_time;
  uint64_t imu_time;

  //ROS_DEBUG("About to do receive.");
  receive(CMD_ACCEL_ANGRATE_MAG, rep, sizeof(rep), 1000, &sys_time);
  //ROS_DEBUG("Receive finished.");

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * G;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the magnetometer reading.
  k = 25;
  for (i = 0; i < 3; i++) {
    mag[i] = extract_float(rep + k);
    k += 4;
  }

  imu_time = extractTime(rep+37);
  *time = filterTime(imu_time, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Receive ACCEL_ANGRATE_ORIENTATION message
void
microstrain_3dmgx2_imu::IMU::receiveAccelAngrateOrientation(uint64_t *time, double accel[3], double angrate[3], double orientation[9])
{
  int i, k;
  uint8_t rep[67];

  uint64_t sys_time;
  uint64_t imu_time;

  //ROS_DEBUG("About to do receive.");
  receive(CMD_ACCEL_ANGRATE_ORIENT, rep, sizeof(rep), 1000, &sys_time);
  //ROS_DEBUG("Finished receive.");

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * G;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the orientation matrix
  k = 25;
  for (i = 0; i < 9; i++) {
    orientation[i] = extract_float(rep + k);
    k += 4;
  }

  imu_time = extractTime(rep+61);
  *time = filterTime(imu_time, sys_time);
}


////////////////////////////////////////////////////////////////////////////////
// Receive ACCEL_ANGRATE message
void
microstrain_3dmgx2_imu::IMU::receiveAccelAngrate(uint64_t *time, double accel[3], double angrate[3])
{
  int i, k;
  uint8_t rep[31];

  uint64_t sys_time;
  uint64_t imu_time;

  receive(CMD_ACCEL_ANGRATE, rep, sizeof(rep), 1000, &sys_time);

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * G;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  imu_time = extractTime(rep+25);
  *time = filterTime(imu_time, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Receive DELVEL_DELANG message
void
microstrain_3dmgx2_imu::IMU::receiveDelvelDelang(uint64_t *time, double delvel[3], double delang[3])
{
  int i, k;
  uint8_t rep[31];

  uint64_t sys_time;
  uint64_t imu_time;

  receive(CMD_DELVEL_DELANG, rep, sizeof(rep), 1000, &sys_time);

  // Read the delta angles:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    delang[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the delta velocities
  k = 13;
  for (i = 0; i < 3; i++)
  {
    delvel[i] = extract_float(rep + k) * G;
    k += 4;
  }

  imu_time = extractTime(rep+25);
  *time = filterTime(imu_time, sys_time);
}


////////////////////////////////////////////////////////////////////////////////
// Receive EULER message
void
microstrain_3dmgx2_imu::IMU::receiveEuler(uint64_t *time, double *roll, double *pitch, double *yaw)
{
  uint8_t rep[19];

  uint64_t sys_time;
  uint64_t imu_time;

  receive(CMD_EULER, rep, sizeof(rep), 1000, &sys_time);

  *roll  = extract_float(rep + 1);
  *pitch = extract_float(rep + 5);
  *yaw   = extract_float(rep + 9);

  imu_time  = extractTime(rep + 13);
  *time = filterTime(imu_time, sys_time);
}
    
////////////////////////////////////////////////////////////////////////////////
// Receive Device Identifier String

bool microstrain_3dmgx2_imu::IMU::getDeviceIdentifierString(id_string type, char id[17])
{
  uint8_t cmd[2];
  uint8_t rep[20];

  cmd[0] = CMD_DEV_ID_STR;
  cmd[1] = type;

  transact(cmd, sizeof(cmd), rep, sizeof(rep), 1000);
  
  if (cmd[0] != CMD_DEV_ID_STR || cmd[1] != type)
    return false;

  id[16] = 0;
  memcpy(id, rep+2, 16);

  if( type==ID_DEVICE_NAME ){
    is_gx3 = (strstr(id,"GX3") != NULL);
  }

  return true;
}

/* ideally it would be nice to feed these functions back into willowimu */
#define CMD_ACCEL_ANGRATE_MAG_ORIENT_REP_LEN 79
#define CMD_RAW_ACCEL_ANGRATE_LEN 31
////////////////////////////////////////////////////////////////////////////////
// Receive ACCEL_ANGRATE_MAG_ORIENT message
void 
microstrain_3dmgx2_imu::IMU::receiveAccelAngrateMagOrientation (uint64_t *time, double accel[3], double angrate[3], double mag[3], double orientation[9]) 
{
	uint8_t	 rep[CMD_ACCEL_ANGRATE_MAG_ORIENT_REP_LEN];

	int k, i;
	uint64_t sys_time;
	uint64_t imu_time;

	receive( CMD_ACCEL_ANGRATE_MAG_ORIENT, rep, sizeof(rep), 1000, &sys_time); 

  // Read the acceleration:
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k) * G;
    k += 4;
  }

  // Read the angular rates
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the magnetic field matrix
  k = 25;
  for (i = 0; i < 3; i++) {
    mag[i] = extract_float(rep + k);
    k += 4;
  }

 // Read the orientation matrix
  k = 37;
  for (i = 0; i < 9; i++) {
    orientation[i] = extract_float(rep + k);
    k += 4;
  }
	imu_time  = extractTime(rep + 73);

	*time = filterTime(imu_time, sys_time);
}

////////////////////////////////////////////////////////////////////////////////
// Receive RAW message
// (copy of receive accel angrate but with raw cmd)
void
microstrain_3dmgx2_imu::IMU::receiveRawAccelAngrate(uint64_t *time, double accel[3], double angrate[3])
{
  int i, k;
  uint8_t rep[CMD_RAW_ACCEL_ANGRATE_LEN];

  uint64_t sys_time;
  uint64_t imu_time;

  receive(microstrain_3dmgx2_imu::IMU::CMD_RAW, rep, sizeof(rep), 1000, &sys_time);

  // Read the accelerator AD register values 0 - 65535 given as float
  k = 1;
  for (i = 0; i < 3; i++)
  {
    accel[i] = extract_float(rep + k);
    k += 4;
  }

  // Read the angular rates AD registor values 0 - 65535 (given as float
  k = 13;
  for (i = 0; i < 3; i++)
  {
    angrate[i] = extract_float(rep + k);
    k += 4;
  }

  imu_time = extractTime(rep+25);
  *time = filterTime(imu_time, sys_time);
}


////////////////////////////////////////////////////////////////////////////////
// Extract time and process rollover
uint64_t
microstrain_3dmgx2_imu::IMU::extractTime(uint8_t* addr)
{
  uint32_t ticks = bswap_32(*(uint32_t*)(addr));

  if (ticks < last_ticks) {
    wraps += 1;
  }

  last_ticks = ticks;

  uint64_t all_ticks = ((uint64_t)wraps << 32) - offset_ticks + ticks;

  return  start_time + (is_gx3 ? (uint64_t)(all_ticks * (1000000000.0 / TICKS_PER_SEC_GX3)) : (uint64_t)(all_ticks * (1000000000.0 / TICKS_PER_SEC_GX2))); // syntax a bit funny because C++ compiler doesn't like conditional ?: operator near the static consts (???)

}



////////////////////////////////////////////////////////////////////////////////
// Send a packet and wait for a reply from the IMU.
// Returns the number of bytes read.
int microstrain_3dmgx2_imu::IMU::transact(void *cmd, int cmd_len, void *rep, int rep_len, int timeout)
{
  send(cmd, cmd_len);
  
  return receive(*(uint8_t*)cmd, rep, rep_len, timeout);
}


////////////////////////////////////////////////////////////////////////////////
// Send a packet to the IMU.
// Returns the number of bytes written.
int
microstrain_3dmgx2_imu::IMU::send(void *cmd, int cmd_len)
{
  int bytes;

  // Write the data to the port
  bytes = write(fd, cmd, cmd_len);
  if (bytes < 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "error writing to IMU [%s]", strerror(errno));

  if (bytes != cmd_len)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "whole message not written to IMU");

  // Make sure the queue is drained
  // Synchronous IO doesnt always work
  if (tcdrain(fd) != 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "tcdrain failed");

  return bytes;
}


static int read_with_timeout(int fd, void *buff, size_t count, int timeout)
{
  ssize_t nbytes;
  int retval;

  struct pollfd ufd[1];
  ufd[0].fd = fd;
  ufd[0].events = POLLIN;

  if (timeout == 0)
    timeout = -1; // For compatibility with former behavior, 0 means no timeout. For poll, negative means no timeout.
  
  if ( (retval = poll(ufd, 1, timeout)) < 0 )
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "poll failed  [%s]", strerror(errno));

  if (retval == 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::TimeoutException, "timeout reached");
	
  nbytes = read(fd, (uint8_t *) buff, count);

  if (nbytes < 0)
    IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "read failed  [%s]", strerror(errno));

  return nbytes;
}

////////////////////////////////////////////////////////////////////////////////
// Receive a reply from the IMU.
// Returns the number of bytes read.
int
microstrain_3dmgx2_imu::IMU::receive(uint8_t command, void *rep, int rep_len, int timeout, uint64_t* sys_time)
{
  int nbytes, bytes, skippedbytes;

  skippedbytes = 0;

  struct pollfd ufd[1];
  ufd[0].fd = fd;
  ufd[0].events = POLLIN;
  
  // Skip everything until we find our "header"
  *(uint8_t*)(rep) = 0;
  
  while (*(uint8_t*)(rep) != command && skippedbytes < MAX_BYTES_SKIPPED)
  {
    read_with_timeout(fd, rep, 1, timeout);

    skippedbytes++;
  }

  if (sys_time != NULL)
    *sys_time = time_helper();
  
  // We now have 1 byte
  bytes = 1;

  // Read the rest of the message:
  while (bytes < rep_len)
  {
    nbytes = read_with_timeout(fd, (uint8_t *)rep + bytes, rep_len - bytes, timeout);
    
    if (nbytes < 0)
      IMU_EXCEPT(microstrain_3dmgx2_imu::Exception, "read failed  [%s]", strerror(errno));
    
    bytes += nbytes;
  }

  // Checksum is always final 2 bytes of transaction

  uint16_t checksum = 0;
  for (int i = 0; i < rep_len - 2; i++) {
    checksum += ((uint8_t*)rep)[i];
  }

  // If wrong throw Exception
  if (checksum != bswap_16(*(uint16_t*)((uint8_t*)rep+rep_len-2)))
    IMU_EXCEPT(microstrain_3dmgx2_imu::CorruptedDataException, "invalid checksum.\n Make sure the IMU sensor is connected to this computer.");
  
  return bytes;
}

////////////////////////////////////////////////////////////////////////////////
// Kalman filter for time estimation
uint64_t microstrain_3dmgx2_imu::IMU::filterTime(uint64_t imu_time, uint64_t sys_time)
{
  // first calculate the sum of KF_NUM_SUM measurements
  if (counter < KF_NUM_SUM){
    counter ++;
    sum_meas += (toDouble(imu_time) - toDouble(sys_time));
  }
  // update kalman filter with fixed innovation
  else{
    // system update
    offset += d_offset;

    // measurement update
    double meas_diff = (sum_meas/KF_NUM_SUM) - offset;
    offset   += KF_K_1 * meas_diff;
    d_offset += KF_K_2 * meas_diff;

    // reset counter and average
    counter = 0; sum_meas = 0;
  }
  return imu_time - toUint64_t( offset ) + toUint64_t( fixed_offset );
}


////////////////////////////////////////////////////////////////////////////////
// convert uint64_t time to double time
double microstrain_3dmgx2_imu::IMU::toDouble(uint64_t time)
{
  double res = trunc(time/1e9);
  res += (((double)time)/1e9) - res;
  return res;
}


////////////////////////////////////////////////////////////////////////////////
// convert double time to uint64_t time
uint64_t  microstrain_3dmgx2_imu::IMU::toUint64_t(double time)
{
  return (uint64_t)(time * 1e9);
}
