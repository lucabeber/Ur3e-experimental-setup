^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rokubimini
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.4 (2020-10-31)
------------------
* Merge branch 'release/0.5.3' into 'melodic-devel'
  Release 0.5.3
  See merge request `botasys/bota_driver!52 <https://gitlab.com/botasys/bota_driver/-/merge_requests/52>`_
* Release 0.5.3
* Merge branch 'fix/cmake-eigen3-error' into 'melodic-devel'
  Try to fix cmake Eigen3 error on ROS buildfarm again
  See merge request `botasys/bota_driver!51 <https://gitlab.com/botasys/bota_driver/-/merge_requests/51>`_
* add different changes
* merge release 0.5.2 into melodic-devel
* Contributors: Mike Karam, Mike Karamousadakis

0.5.3 (2020-10-30)
------------------
* Merge branch 'fix/cmake-eigen3-error' into 'melodic-devel'
  Try to fix cmake Eigen3 error on ROS buildfarm again
  See merge request `botasys/bota_driver!51 <https://gitlab.com/botasys/bota_driver/-/merge_requests/51>`_
* add different changes
* merge release 0.5.2 into melodic-devel
* Contributors: Mike Karam, Mike Karamousadakis

0.5.5 (2020-11-02)
------------------
* Merge branch 'release/0.5.4' into 'melodic-devel'
  Release 0.5.4
  See merge request botasys/bota_driver!54
* Release 0.5.4
* Merge branch 'release/0.5.3' into 'melodic-devel'
  Release 0.5.3
  See merge request botasys/bota_driver!52
* Release 0.5.3
* Merge branch 'fix/cmake-eigen3-error' into 'melodic-devel'
  Try to fix cmake Eigen3 error on ROS buildfarm again
  See merge request botasys/bota_driver!51
* add different changes
* merge release 0.5.2 into melodic-devel
* Contributors: Mike Karam, Mike Karamousadakis

0.5.2 (2020-10-29)
------------------
* Merge branch 'fix/cmake-eigen3-error' into 'master'
  Fix eigen3 cmake error on ROS buildfarm
  See merge request `botasys/bota_driver!49 <https://gitlab.com/botasys/bota_driver/-/merge_requests/49>`_
* fix eigen3 cmake error on ROS buildfarm
* Merge branch 'feature/increment-patch-version' into 'master'
  Increment patch version
  See merge request `botasys/bota_driver!48 <https://gitlab.com/botasys/bota_driver/-/merge_requests/48>`_
* Increment patch version
* Contributors: Mike Karam, Mike Karamousadakis

0.5.1 (2020-10-27)
------------------

0.5.6 (2020-11-03)
------------------
* Support clang format 8
* Contributors: Mike Karamousadakis

0.5.7 (2020-11-04)
------------------
* Fix gcc error for extended alignment
* Release master 0.5.6
* Support clang format 8
* Contributors: Mike Karam, Mike Karamousadakis

0.5.8 (2020-11-13)
------------------
* Release 0.5.7 into master
* add condition for adding -faligned-new flag in GCC
* Contributors: Mike Karam, Mike Karamousadakis

0.5.9 (2021-02-06)
------------------
* add url to wiki
* Feature - add firmware update
* Feature - add unit testing of rokubimini
* fix catkin_lint warning: variable CMAKE_CXX_FLAGS is modified
* Contributors: Mike Karam, Mike Karamousadakis

0.6.0 (2021-06-22)
------------------
* New launch interface - the topology is in the launch file
* Contributors: Mike Karam, Mike Karamousadakis

0.6.1 (2021-09-28)
------------------
* Set imu angular rate.
* add missing files in install space
* Contributors: Martin, Mike Karam, Mike Karamousadakis

0.5.0 (2020-10-20)
------------------
* Merge branch 'feature/move-publishers-to-instances'
* Merge branch 'feature/remove-message-logger-dependency'
* Merge branch 'feature/rename-project' into 'master'
  Rename project to bota_driver
* Merge branch 'feature/remove-yaml-tools-dep' into 'master'
  Remove yaml tools dependency
* Merge branch 'feature/remove-cosmo-dependency' into 'master'
  Remove all dependencies apart from yaml_tools
* Merge branch 'feature/save-config-parameter'
* Merge branch 'feature/add-temperature-serial'
* Merge branch 'feature/add-si-units-imu'
* add temperature publishing functionality. tested locally
* add first implementation. works on Serial, but it's still WIP
* add conversion to SI units for IMU data
* Merge branch 'feature/remove-command-from-repo' into 'master'
  remove Command from repo. Tested locally
* Merge branch 'feature/change-printed-messages' into 'master'
  Feature/change printed messages
* Initial commit
  - change [ForceTorqueSensor::device_name] to [device_name] in RokubiminiSerialImpl.cpp
  - add bus name and/or device_name in RokubiminiEthercatSlave, RokubiminiEthercatBusManager and EthercatBusBase.
* Merge branch 'add-abstract-filters' into 'master'
  Add configuration support in serial devices
* fix bug in assignment operation of Configuration. replace INIT_MODE with ConnectionState in RokubiminiSerialImpl code
* Merge branch 'add-linter-test' into 'feature/rokubimini_serial'
  Add linter testing step in CI
* add clang-formated code. add support for multiple devices in rokubimini_cosmo
* add doxygen documentation
* first abstracted try
* Merged in feature/temperature_reading (pull request #31)
  add temperature in readings
  Approved-by: Johannes Pankert <johannes@pankert.eu>
* add temperature in readings
* remove redundant orientation. Imu has quaternion
* Merged in feature/publish_standard_ros_msgs (pull request #29)
  remove redundant orientation. Imu has quaternion
  Approved-by: Johannes Pankert <johannes@pankert.eu>
  Approved-by: Martin Wermelinger <martiwer@mavt.ethz.ch>
* remove redundant orientation. Imu has quaternion
* Merge branch 'master' into feature/publish_rokubimini_reading
* Merged in Feature/default_config (pull request #20)
  Feature/default config
  Approved-by: Markus Stäuble <markus.staeuble@mavt.ethz.ch>
  Approved-by: Johannes Pankert <johannes@pankert.eu>
* add force torque offset support in config
* add IMU filter config and correct range config
* Merged in bugfix/error_reporting (pull request #19)
  Bugfix/error reporting
  Approved-by: Markus Stäuble <markus.staeuble@mavt.ethz.ch>
  Approved-by: Johannes Pankert <johannes@pankert.eu>
* add serial number readout
* add option to run sync with the FT sample rate
* add force torque filter configuration
* print statusword only on DEBUG mode
* correct error reporting
* Merge branch 'master' into fix/base_sensor_with_2_adapter_plates
* Merged in feature/calibration_only (pull request #16)
  add calibration of the F/T sensor
  Approved-by: Martin Wermelinger <martiwer@mavt.ethz.ch>
  Approved-by: Johannes Pankert <johannes@pankert.eu>
  Approved-by: Dario Nastasi <nastasid@student.ethz.ch>
* moved doxygen method descriptions to header files
* formatting
* formatting
* add calibration of the F/T sensor
* Merge branch 'feature/ft_dario_testing' into feature/dario_rafael_master_v2
* Removed dep
* Another test
* Removed test dep
* Testing with external dep
* another attempt to prevent memory error when closing simulation
* fix memory error when closing simulation
* apply all changes from force_torque_controllers without ethercat changes
* Merged in feature/OD_clean_up (pull request #8)
  Feature/OD clean up
  Approved-by: Markus Stäuble <markus.staeuble@mavt.ethz.ch>
* Adressed pr comments
* Working with newest firmware
* wp 2
* Adapted to latest yaml node changes
* Merged in feature/calibration (pull request #5)
  Feature/calibration
* Removed 2 unused warnings
* Calibration matrix can now be set with sdo
* Added imu ranges to sdo
* Reading calibration matrix from file
* Deleted driver, added more sdos
* to be tested with new firmware
* Trying to write calibration sdo
* Set up the basic structure to run calibration sdo
* another melo include
* Added melo include
* Updated documentation
* Merged in feature/manager (pull request #3)
  Feature/manager
* Started implementing manager classes
* Clang tools
* Added ros conversion trait for reading and command
* Cosmo example back to life
* Started to adapt cosmo example
* Configuration SDO works
* Added first sdo
* Wip to make example master work
* Example compiles
* Wip example, file support, bug fixes
* Rokubimini_ethercat compiles
* Rokubimini compiles, added command
* Wip to make things run
* Wip to implement new structure
* Contributors: Ilias Patsiaouras, Mike Karamousadakis
