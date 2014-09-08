/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-2010  Willow Garage
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


#ifndef MS_3DMGX2_HH
#define MS_3DMGX2_HH

#include <fstream>
#include <stdexcept>
#include <stdint.h>

namespace microstrain_3dmgx2_imu
{

  //! Macro for defining exception (std::runtime_error should be top parent)
  #define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name(const char* msg) : parent(msg) {} \
  }

  DEF_EXCEPTION(Exception, std::runtime_error);
  DEF_EXCEPTION(TimeoutException, Exception);
  DEF_EXCEPTION(CorruptedDataException, Exception);

  #undef DEF_EXCEPTION

  //! A class for interfacing to the microstrain 3dmgx2 and inertialink IMUs
  /*!
   * Note: This class is unreviewed and unsupported. It may change at any
   * time without notice.
   *
   * Many of the methods within this class may throw an
   * microstrain_3dmgx2_imu::exception, timeout_exception, or
   * corrupted_data_exception.
   *
   * Before using the IMU, it must be opened via the open_port method.
   * When finished using, it should be closed via the close_port
   * method.  Alternatively, close port will get called at
   * destruction.
   *
   * The library is primarily designed to be used in continuous mode,
   * which is enabled with the set_continuous method, and then serviced
   * with one of the receive methods.
   *
   * Implementation of specific polled message transactions can be
   * done with the transact method.
   *
   * Because the timing related to the USB stack is fairly
   * non-deterministic, but the IMU is internally known to be clocked
   * to a 100hz clock, we have wrapped a Kalman filter around calls to
   * get system time, and the internal imu time.  This is only known
   * to be reliable when operating in continuous mode, and if init_time
   * is called shortly prior to beginning to get readings.
   * 
   *
   * Example code:
   * \code
   *   microstrain_3dmgx2_imu::IMU imu;
   *   imu.open_port("/dev/ttyUSB0");
   *   imu.init_time();
   *   imu.init_gyros();
   *   imu.set_continuous(microstrain_3dmgx2_imu::IMU::CMD_ACCEL_ANGRATE_ORIENT);
   *   while (int i = 0 ; i < 100; i++)
   *   {
   *     double accel[3];
   *     double angrate[3];
   *     double orientation[9];
   *     imu.receive_accel_angrate_orientation(&time, accel, angrate, orientation);
   *   }
   *   imu.close_port();
   * \endcode
   */
  class IMU
  {
    //! IMU internal ticks/second
    static const int TICKS_PER_SEC_GX2  = 19660800;
    static const int TICKS_PER_SEC_GX3  = 62500;
    //! Maximum bytes allowed to be skipped when seeking a message
    static const int MAX_BYTES_SKIPPED  = 1000;
    //! Number of KF samples to sum over
    static const unsigned int KF_NUM_SUM= 100;
    //! First KF term
    static const double KF_K_1          = 0.00995031;
    //! Second KF term
    static const double KF_K_2          = 0.0000497506;

  public: 

    //! Gravity (m/sec^2)
    static const double G               = 9.80665;    

    //! Enumeration of possible IMU commands
    enum cmd {
      CMD_RAW                      =  0xC1,
      CMD_ACCEL_ANGRATE            =  0xC2,
      CMD_DELVEL_DELANG            =  0xC3,
      CMD_CONTINUOUS               =  0xC4,
      CMD_ORIENT                   =  0xC5,
      CMD_ATT_UPDATE               =  0xC6,
      CMD_MAG_VEC                  =  0xC7,
      CMD_ACCEL_ANGRATE_ORIENT     =  0xC8,
      CMD_WRITE_ACCEL_BIAS         =  0xC9,
      CMD_WRITE_GYRO_BIAS          =  0xCA,
      CMD_ACCEL_ANGRATE_MAG        =  0xCB,
      CMD_ACCEL_ANGRATE_MAG_ORIENT =  0xCC,
      CMD_CAPTURE_GYRO_BIAS        =  0xCD,
      CMD_EULER                    =  0xCE,
      CMD_EULER_ANGRATE            =  0xCF,
      CMD_TEMPERATURES             =  0xD1,
      CMD_GYROSTAB_ANGRATE_MAG     =  0xD2,
      CMD_DELVEL_DELANG_MAG        =  0xD3,
      CMD_DEV_ID_STR               =  0xEA,
      CMD_STOP_CONTINUOUS          =  0xFA
    };

    //! Enumeration of possible identifier strings for the getDeviceIdentifierString command.

    enum id_string {
      ID_MODEL_NUMBER   = 0,
      ID_SERIAL_NUMBER  = 1,
      ID_DEVICE_NAME    = 2,
      ID_DEVICE_OPTIONS = 3
    };

    //! Constructor
    IMU();

    // Destructor
    ~IMU();

    //! Open the port
    /*! 
     * This must be done before the imu can be used.
     * 
     * \param port_name   A character array containing the name of the port
     *
     */
    void openPort(const char *port_name);

    //! Close the port
    void closePort();

    //! Initialize timing variables.
    /*!
     * This call determines the initial offset of the imu relative to 
     * system clock time, and resets the kalman filter state.
     *
     * \param fix_off this fixed offset will be added to the timestamp of the imu
     */
    void initTime(double fix_off);

    //! Initial gyros
    /*! 
     * This call will prompt the IMU to run its gyro initialization
     * routine.  
     *
     * NOTE: THE IMU MUST BE STATIONARY WHEN THIS ROUTINE IS CALLED
     *
     * \param bias_x   Pointer to double where x bias will be placed.
     * \param bias_y   Pointer to double where y bias will be placed.
     * \param bias_z   Pointer to double where z bias will be placed.
     */
    void initGyros(double* bias_x = 0, double* bias_y = 0, double* bias_z = 0);

    //! Put the device in continuous mode
    /*!
     * This call puts the IMU into a mode where it is continuously
     * outputting a particular message.
     *
     * \param command   The type of message to be output.
     * 
     * \return  Whether or not continuous mode was enabled successfully.
     */
    bool setContinuous(cmd command);

    //! Take the device out of continous mode.
    void stopContinuous();

    //! Read a message of type "ACCEL_ANGRATE"
    /*! 
     * \param time    Pointer to uint64_t which will receive time
     * \param accel   array of accelerations which will be filled
     * \param angrate array of angular rates which will be filled
     */
    void receiveAccelAngrate(uint64_t *time, double accel[3], double angrate[3]);

    //! Read a message of type "DELVEL_DELANG"
    /*! 
     * \param time    Pointer to uint64_t which will receive time
     * \param delvel array of accelerations which will be filled
     * \param delang array of angular rates which will be filled
     */
    void receiveDelvelDelang(uint64_t *time, double delvel[3], double delang[3]);

    //! Read a message of type "ACCEL_ANGRATE_MAG"
    /*! 
     * \param time    Pointer to uint64_t which will receive time
     * \param accel   array of accelerations which will be filled
     * \param angrate array of angular rates which will be filled
     * \param mag     array of magnetometer orientations which will be filled
     */
    void receiveAccelAngrateMag(uint64_t *time, double accel[3], double angrate[3], double mag[3]);

    //! Read a message of type "EULER"
    /*! 
     * \param time    Pointer to uint64_t which will receive time
     * \param roll    Pointer to roll value which will be filled
     * \param pitch   Pointer to pitch value which will be filled
     * \param yaw     Pointer to yaw value which will be filled
     */
    void receiveEuler(uint64_t *time, double *roll, double *pitch, double *yaw);

    //! Read a message of type "ACCEL_ANGRATE_ORIENTATION"
    /*! 
     * \param time        Pointer to uint64_t which will receive time
     * \param accel       array of accelerations which will be filled
     * \param angrate     array of angular rates which will be filled
     * \param orientation orientation matrix which will be filled
     */
    void receiveAccelAngrateOrientation(uint64_t *time, double accel[3], double angrate[3], double orientation[9]);

	//! Read a message of type "ACCEL_ANGRATE_MAG_ORIENT"
	/*! 
	 * \param time    Pointer to uint64_t which will receive time
	 * \param accel   array of accelerations which will be filled
	 * \param angrate array of angular rates which will be filled
	 * \param mag     array of magnetometer orientations which will be filled
	 * \param orientation orientation matrix which will be filled
	 */
	void receiveAccelAngrateMagOrientation (uint64_t *time, double accel[3], double angrate[3], double mag[3], double orientation[9]);

	//! Read a message of type "CMD_RAW"
	/*! 
	 * \param time    Pointer to uint64_t which will receive time
	 * \param accel   array of accelerations which will be filled
	 * \param angrate array of angular rates which will be filled
	 */
	void receiveRawAccelAngrate(uint64_t *time, double accel[3], double angrate[3]);

    //! Set the fixed time offset
    /*! 
     * \param fix_off  Fixed time offset in seconds
     */
    void setFixedOffset(double fix_off) {fixed_offset = fix_off;};

    //! Read one of the device identifier strings
    /*!
     * \param type Indicates which identifier string to read
     * \param id Array that gets filled with the identifier string
     * \return True if successful
     */
    bool getDeviceIdentifierString(id_string type, char id[17]);

  private:
    //! Send a command to the IMU and wait for a reply
    int transact(void *cmd, int cmd_len, void *rep, int rep_len, int timeout = 0);

    //! Send a single packet frmo the IMU
    int send(void *cmd, int cmd_len);

    //! Receive a particular message from the IMU
    int receive(uint8_t command, void *rep, int rep_len, int timeout = 0, uint64_t* sys_time = NULL);

    //! Extract time from a pointer into an imu buffer
    uint64_t extractTime(uint8_t* addr);

    //! Run the filter on the imu time and system times
    uint64_t filterTime(uint64_t imu_time, uint64_t sys_time);

    //! Convert the uint64_t time to a double for numerical computations
    double toDouble(uint64_t time);

    //! Convert the double time back to a uint64_t
    uint64_t toUint64_t(double time);

    //! The file descriptor
    int fd;

    //! The number of times the imu has wrapped
    uint32_t wraps;

    //! The number of ticks the initial offset is off by
    uint32_t offset_ticks;

    //! The last number of ticks for computing wraparound
    uint32_t last_ticks;

    //! The different in the number of ticks
    uint32_t diff_ticks;

    //! The time at which the imu was started
    unsigned long long start_time;

    //! The estimate of time offset and driftrate
    double time_est[2];

    //! The covariances on time offset and driftrate
    double P_time_est[2][2];

    //! Whether continuous mode is enabled
    bool continuous;

    //! A counter used by the filter
    unsigned int counter;

    //! Variables used by the kalman computation
    double fixed_offset, offset, d_offset, sum_meas;

    //! Is the IMU a GX3?
    bool is_gx3;

  };

}
#endif
