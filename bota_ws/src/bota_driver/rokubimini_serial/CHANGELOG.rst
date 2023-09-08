^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rokubimini_serial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.4 (2020-10-31)
------------------
* Merge branch 'fix/printf-breaks-build-arm' into 'melodic-devel'
  Fix ROS_DEBUG() breaking ARM build on ROS buildfarm
  See merge request `botasys/bota_driver!53 <https://gitlab.com/botasys/bota_driver/-/merge_requests/53>`_
* change %lu to %zu
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
* Merge branch 'fix/printf-breaks-build-arm' into 'melodic-devel'
  Fix ROS_DEBUG() breaking ARM build on ROS buildfarm
  See merge request botasys/bota_driver!53
* change %lu to %zu
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
* Contributors: Mike Karamousadakis

0.5.1 (2020-10-27)
------------------

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
* Release master 0.5.6
* change minimum cmake version required to 3.0.2
* Support clang format 8
* Contributors: Mike Karamousadakis

0.5.8 (2020-11-13)
------------------


0.5.9 (2021-02-06)
------------------
* add url to wiki
* Feature - add reset services
* Feature - add firmware update
* Separated adapter from urdf and changed publishing topic names in bota_driver.
* fix bug on startup related to modeState
* Contributors: Ilias Patsiaouras, Lefteris Kotsonis, Mike Karamousadakis

0.6.0 (2021-06-22)
------------------
* Fix bugs
* Acknowledgement of serial commands
* Parsing of product name and boot messages of serial sensor
* Publishing based on the timestamp of serial sensor
* Automatically change to the maximum available baud rate
* New launch interface - the topology is in the launch file
* Feature - add format of commands based on command classes
* Feature - add regex and command classes
* Fix - mean wrench before reset
* Contributors: Ilias Patsiaouras, Lefteris Kotsonis, Mike Karamousadakis

0.6.1 (2021-09-28)
------------------
* Throttle error message, set max attempts to open serial port.
* remove shutting down because of too many timeouts
* Handle time stamp as uint and not float
* add missing files in install space
* Show serial number in boot
* Contributors: Ilias Patsiaouras, Lefteris Kotsonis, Martin Wermelinger, Mike Karam, Mike Karamousadakis

0.5.0 (2020-10-20)
------------------
* Merge branch 'feature/move-publishers-to-instances'
* Merge branch 'feature/remove-message-logger-dependency'
* Merge branch 'feature/enable-configuration-object-serial' into 'master'
  Enable configuration object in serial
* Merge branch 'fix/add-charachter_delay' into 'master'
  add inter-character delay 5ms
* Merge branch 'feature/publish-frame-id' into 'master'
  Add frame_id in published data
* Merge branch 'feature/rename-project' into 'master'
  Rename project to bota_driver
* Merge branch 'feature/remove-serial-types-file' into 'master'
  Remove rokubimini_serial/types.hpp file
* Merge branch 'feature/remove-yaml-tools-dep' into 'master'
  Remove yaml tools dependency
* Merge branch 'feature/remove-cosmo-dependency' into 'master'
  Remove all dependencies apart from yaml_tools
* Merge branch 'feature/add-init-step-serial'
* rename setSoftwareReset() to setInitMode()
* Merge branch 'feature/save-config-parameter'
* add setSoftwareReset() in init() of serial
* Merge branch 'feature/add-temperature-serial'
* Merge branch 'feature/add-si-units-imu'
* Merge branch 'feature/add-calibration-matrix-command'
* reduce number of chars to 62 with %9.6f
* add proposed changes
* add temperature publishing functionality. tested locally
* add first implementation. works on Serial, but it's still WIP
* fix mispelled frame name
* add temperature in serial Reading
* add conversion to SI units for IMU data
* add RokubiminiSerialImpl::setSensorCalibration() command. Tested locally.
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
* Merge branch 'add-baudrate-115200' into 'master'
  add missing baudrate
* Merge branch 'change-info-to-debug' into 'master'
  change some INFO to DEBUG
* Merge branch 'add-linter-test' into 'feature/rokubimini_serial'
  Add linter testing step in CI
* add baud rate functionality: baud rate is taken from the system definitions (termios.h in Linux)
* add clang-formated code. add support for multiple devices in rokubimini_cosmo
* add doxygen documentation
* fix CI build
* first abstracted try
* Contributors: Ilias Patsiaouras, Mike Karamousadakis
