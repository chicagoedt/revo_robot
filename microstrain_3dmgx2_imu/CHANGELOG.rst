^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package microstrain_3dmgx2_imu
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.12 (2014-01-15)
-------------------
* Added dependency on log4cxx.
* Contributors: Chad Rockey

1.5.10 (2013-07-23)
-------------------
* Reduced console spam when shutting down, extended time to wait for ros::isShuttingDown()

1.5.9 (2013-04-16 14:03)
------------------------
* Added depdency on _gencpp for service.

1.5.8 (2013-04-16 11:40)
------------------------
* Forgot srv directory in CMakeLists.txt

1.5.7 (2013-04-04 18:02)
------------------------
* Accidentally copied message_generation instead of message_runtime

1.5.6 (2013-04-04 17:17)
------------------------
* Adding std_msgs to depends.

1.5.5 (2013-04-04 16:33)
------------------------
* Fixed message_generation.  Standardized CMakeLists.txt

1.5.4 (2013-04-03)
------------------
* Fixed install targets.
* Update package.xml

1.5.3 (2013-03-26)
------------------
* Added install rule for other files.
* Added message if log4xx not found
* Updated version to 1.5.2
* Catkinized microstrain driver.
* Updated initTime() to be called before calibrate.
* Fix for calibration error on some devices.
* oneiric fix
* Updated to republish calibration status if a new calibration fails.
* Ran tf bullet types migration script.
* applying patch provided by RobZ
* adding support for the 3DM-GX3 protocol
* Now opens IMU with NOCTTY to avoid getting signaled if the serial port dies. Moved O_NONBLOCK to the correct position in the open call, could have caused problems when accessed by multiple programs otherwise (one of which wasn't locking).
* Fixed spacing in quiet mode for IMU get_id utility. `#4287 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/4287>`_
* Quiet mode for get_id utility. `#4287 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/4287>`_
* Spelling for IMU get ID function. `#4287 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/4287>`_
* IMU utility to get device ID. `#4287 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/4287>`_
* Added Ubuntu platform tags to manifest
* Fixed linker export in manifest.
* add option to assume calibrated status on startup, to allow no-calibration
* make calibration routine check for drift after it finished
* remove deprecated subscriptions
* Changed frameid frame_id. Orientation is now from the world frame to the imu frame. A 180 degree rotation around the y axis has been added so that the world frame z vector points up.
* Tweaked error messages.
* Marking imu_drivers as doc reviewed.
* Added 'this is unstable' notices to the doxygen.
* Took out call to diagnostics update when we are in error. The error broadcasts will take care of publishing the diagnostics in that case.
* Made a few error messages more useful if you are not connected to an IMU or if there is a communication glitch.
* Reworked error reporting. Added locking of the port. Made IO non-blocking to avoid race conditions between multiple reading processes. Closes tickets `#1442 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/1442>`_ `#3188 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/3188>`_ `#2342 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/2342>`_ `#3356 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/3356>`_ `#3508 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/3508>`_ `#3507 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/3507>`_.
* Switched imu_node.cc back to BSD only as it is less restrictive than LGPL.
* Updated all licences to LGPL.
* Updated stack documentation and license.
* Tweaked doxygen main page.
* Moved doxygen stub out of imu_node.cc
* Renamed imu_data to imu, and pushed the main data topic into imu/data.
* Corrected node name in microstrain_3dmgx2.launch
* Fixed up doxygen.
* Added -Wl,-rpath,-L/lib to cpp exports as requested by `#3479 <https://github.com/ros-drivers/microstrain_3dmgx2_imu/issues/3479>`_.
* staging imu_drivers into tick-tock
* Fixed the calibrate service call.
* Marked as API cleared.
* Got imu_node up to date with api changes in diagnostic_updater and self_test. Now sets HardwareID in diagnostics. Extended the ID that is returned by the self-test. Got rid of the thread that was previously needed just for self_test synchronization.
* should not be blacklisted
* Finished merging 3dmgx2 and imu_node. Time to deprecate the old packages.
* Moved is_calibrated and calibrate into imu_data name space instead of ~ name space.
* Merged imu_node and 3dmgx2 into microstrain_3dmgx2_imu. Still need to fix the package references in the files.
* Created the microstrain_3dmgx2_imu package to subsume the imu_node and 3dmgx2_driver packages.
