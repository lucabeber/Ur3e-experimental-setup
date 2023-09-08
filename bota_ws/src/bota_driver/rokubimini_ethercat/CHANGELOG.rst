^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rokubimini_ethercat
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.4 (2020-10-31)
------------------
* Merge branch 'release/0.5.3' into 'melodic-devel'
  Release 0.5.3
  See merge request `botasys/bota_driver!52 <https://gitlab.com/botasys/bota_driver/-/merge_requests/52>`_
* Release 0.5.3
* merge release 0.5.2 into melodic-devel
* Contributors: Mike Karam, Mike Karamousadakis

0.5.3 (2020-10-30)
------------------
* merge release 0.5.2 into melodic-devel
* Contributors: Mike Karam

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
* merge release 0.5.2 into melodic-devel
* Contributors: Mike Karam, Mike Karamousadakis

0.5.2 (2020-10-29)
------------------
* Merge branch 'feature/increment-patch-version' into 'master'
  Increment patch version
  See merge request `botasys/bota_driver!48 <https://gitlab.com/botasys/bota_driver/-/merge_requests/48>`_
* Increment patch version
* Merge branch 'feature/remove-any-node-dep' into 'master'
  Remove any_node dependency
  See merge request `botasys/bota_driver!47 <https://gitlab.com/botasys/bota_driver/-/merge_requests/47>`_
* Remove any_node dependency
* Contributors: Mike Karamousadakis

0.5.1 (2020-10-27)
------------------
* Merge branch 'feature/remove-any-node-dep' into 'master'
  Remove any_node dependency
  See merge request botasys/bota_driver!47
* Remove any_node dependency
* Contributors: Mike Karamousadakis

0.5.6 (2020-11-03)
------------------
* Merge branch 'fix/cmake-policy-cmp0048' into 'master'
  change minimum cmake version required to 3.0.2
  Closes #29
  See merge request botasys/bota_driver!66
* change minimum cmake version required to 3.0.2
* Support clang format 8
* Contributors: Mike Karamousadakis

0.5.7 (2020-11-04)
------------------
* Fix gcc error for extended alignment
* Release master 0.5.6
* change minimum cmake version required to 3.0.2
* Support clang format 8
* Contributors: Mike Karamousadakis

0.5.8 (2020-11-13)
------------------
* Release 0.5.7 into master
* add condition for adding -faligned-new flag in GCC
* Contributors: Mike Karamousadakis

0.5.9 (2021-02-06)
------------------
* add url to wiki
* Move soem_interface to rokubimini namespace.
* Feature - add reset services
* Feature - add firmware update
* Separated adapter from urdf and changed publishing topic names in bota_driver.
* fix catkin_lint warning: variable CMAKE_CXX_FLAGS is modified
* Contributors: Ilias Patsiaouras, Lefteris Kotsonis, Martin, Mike Karamousadakis

0.6.0 (2021-06-22)
------------------
* Fix - mean wrench before reset
* New launch interface - the topology is in the launch file
* Fix bugs
* Contributors: Ilias Patsiaouras, Lefteris Kotsonis, Martin, Mike Karamousadakis

0.6.1 (2021-09-28)
------------------
* Feature/ethercat grant
* add missing files in install space
* fix S/N wrongly displayed in ethercat
* Show serial number in boot
* Contributors: Ilias Patsiaouras, Lefteris Kotsonis, Martin, Martin Wermelinger, Mike Karamousadakis

0.5.0 (2020-10-20)
------------------
* Merge branch 'feature/remove-redundant-ethercat-pdos'
* Merge branch 'feature/move-publishers-to-instances'
* Merge branch 'feature/remove-message-logger-dependency'
* revert some changes
* Merge branch 'fix/fix-flag-isforcetorquesaturated'
* remove message_logger from docs + CI
* fix clang format
* Merge branch 'feature/fix-catkin-make' into 'master'
  Fix broken catkin_make
* Fix broken catkin_make
* Merge branch 'feature/publish-frame-id' into 'master'
  Add frame_id in published data
* Merge branch 'feature/rename-project' into 'master'
  Rename project to bota_driver
* add frame_id in published data
* Merge branch 'feature/remove-yaml-tools-dep' into 'master'
  Remove yaml tools dependency
* Merge branch 'feature/remove-cosmo-dependency' into 'master'
  Remove all dependencies apart from yaml_tools
* Merge branch 'feature/add-init-step-serial'
* Merge branch 'feature/save-config-parameter'
* remove redundant file
* add STATUS check
* Merge branch 'feature/add-temperature-serial'
* Merge branch 'feature/add-si-units-imu'
* add temperature publishing functionality. tested locally
* add first implementation. works on Serial, but it's still WIP
* fix clang format
* add changes to every PdoType
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
* fix CI build
* remove soem_interface dependancy
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
* removed explicit soem dependency
* Merge branch 'master' into feature/publish_rokubimini_reading
* Merged in Feature/default_config (pull request #20)
  Feature/default config
  Approved-by: Markus Stäuble <markus.staeuble@mavt.ethz.ch>
  Approved-by: Johannes Pankert <johannes@pankert.eu>
* add force torque offset support in config
* move some info to debug level
* add IMU filter config and correct range config
* add serial number readout
* add option to run sync with the FT sample rate
* add force torque filter configuration
* Merge remote-tracking branch 'origin/master' into feature/dario_rafael_master_v2
  # Conflicts:
  #	rokubimini_description/urdf_src/include/R212_parameters.urdf.xacro
  #	rokubimini_description/urdf_src/roku_force_torque_sensor.urdf.xacro
  #	rokubimini_gazebo_plugin/config/default.yaml
  #	rokubimini_gazebo_plugin/include/rokubimini_gazebo_plugin/RokubiminiGazeboPlugin.h
  #	rokubimini_gazebo_plugin/src/RokubiminiGazeboPlugin.cpp
* Merged in feature/euthing_sensor (pull request #11)
  modified TxPDO C to work with euthing feet
  Approved-by: Markus Stäuble <markus.staeuble@mavt.ethz.ch>
* modified TxPDO C to work with it
* Merged in feature/OD_clean_up (pull request #8)
  Feature/OD clean up
  Approved-by: Markus Stäuble <markus.staeuble@mavt.ethz.ch>
* Merged in feature/daisy_chain (pull request #9)
  Feature/daisy chain
* Merge branch 'feature/daisy_chain' into feature/OD_clean_up
* Adressed pr comments
* Working with newest firmware
* wp 2
* wp
* Improved error message
* Clean up, improved methods for daisy chaining
* Merged in feature/unique_ptr (pull request #6)
  Adapted to base class changes, added method to add rokubimini slaves to an existing bus
* Adapted to base class changes, added method to add rokubimini slaves to an existing bus
* Merged in feature/calibration (pull request #5)
  Feature/calibration
* Calibration matrix can now be set with sdo
* Merge branch 'master' into feature/calibration
* Merged in feature/bare_ptr (pull request #4)
  Feature/bare ptr
* Adapted pdo to latest object dictionary changes
* Adapted to soem interface change
* Added imu ranges to sdo
* Reading calibration matrix from file
* Deleted driver, added more sdos
* to be tested with new firmware
* Trying to write calibration sdo
* Set up the basic structure to run calibration sdo
* MONSTER COMMIT!!!!!!!!!!!!!!!!!!!!!
* Updated documentation
* Merged in feature/manager (pull request #3)
  Feature/manager
* Added external imu pdo to use for THING
* Got rid of absolute path in example
* Manager works now with cosmo example
* Fix cmake
* Further manager development
* Cosmo node compiles with manager
* Started implementing manager classes
* Using more shared ptr now
* Clang tools
* Adapted to latest changes, clean up
* Added ros conversion trait for reading and command
* Cosmo example back to life
* Started to adapt cosmo example
* Configuration SDO works
* Added first sdo
* Master working properly with the size check
* Added more PDOs
* Working master example
* Wip to make example master work
* Example compiles
* Wip example, file support, bug fixes
* Rokubimini_ethercat compiles
* Rokubimini compiles, added command
* Wip to make things run
* Wip to implement new structure
* Contributors: Ilias Patsiaouras, Mike Karamousadakis
