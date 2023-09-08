^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rokubimini_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Mike Karamousadakis

0.5.7 (2020-11-04)
------------------
* Release master 0.5.6
* change minimum cmake version required to 3.0.2
* Contributors: Mike Karamousadakis

0.5.8 (2020-11-13)
------------------

0.5.9 (2021-02-06)
------------------
* add url to wiki
* fix old path
* Delete old urdf files.
* Added realsense urdf.
* Added rviz config.
* Added sensone ocfigurations.
* Added all the rokubi configurations in the urdf.
* Added urdf for sensone.
* Moved meshes into rokubimini_description.
* Separated adapter from urdf and changed publishing topic names in bota_driver.
* Added rokubi urdf form solidworks
* Contributors: Lefteris Kotsonis, Mike Karamousadakis

0.6.0 (2021-06-22)
------------------

0.6.1 (2021-09-28)
------------------
* Fix/simplify collision mesh
* Contributors: Lefteris Kotsonis, Martin Wermelinger, Mike Karamousadakis

0.5.0 (2020-10-20)
------------------
* Merge branch 'feature/remove-message-logger-dependency'
* Merge branch 'add-linter-test' into 'feature/rokubimini_serial'
  Add linter testing step in CI
* double checked tensor, mass and collision cylinder, and formatting
* formatting
* added inertia tensor, mass, collision cylinder
* formatting
* added inertia to parameter file
* Set inertial rpy to zero.
* Added default config file for rokubimega.
* changed sensor coordinate system. updated rokubimega.stl file
* edit FT parameters for Shovel and Gripper mount
* added stl file in mashes, created and adapted parameter and force_torque_sensors files
* Merged in feature/rokubimega (pull request #27)
  Feature/rokubimega
  Approved-by: Martin Wermelinger <martiwer@mavt.ethz.ch>
  Approved-by: Dominic Jud <djud@student.ethz.ch>
  Approved-by: Ilias Patsiaouras <ilias.patsiaouras@gmail.com>
* double checked tensor, mass and collision cylinder, and formatting
* formatting
* added inertia tensor, mass, collision cylinder
* formatting
* Merge branch 'feature/rokubimega' of bitbucket.org:leggedrobotics/rokubimini_ethercat_sdk into feature/rokubimega
* Set inertial rpy to zero.
* Merge branch 'feature/rokubimega' of bitbucket.org:leggedrobotics/rokubimini_ethercat_sdk into feature/rokubimega
* added inertia to parameter file
* Added default config file for rokubimega.
* changed sensor coordinate system. updated rokubimega.stl file
* edit FT parameters for Shovel and Gripper mount
* added stl file in mashes, created and adapted parameter and force_torque_sensors files
* moved sensor frame such that the RKL100 urdf accounts for the second adapter plate
* Merge branch 'feature/fix_simulated_sensor_joint'
* changed standard value of fix_sensor in urdf
* Merged in feature/fix_simulated_sensor_joint (pull request #24)
  Allow to fix sensor joint when simulation flag is set to true
* removed simulation argument from sensor_fixed conditional expression
* the sensor joint type can be set to fixed even when the simulation flag is true. This allows for passing the robot description with fixed sensor joints to the robot_state_publisher.
* Merged in fix/ee_sensor_parameters (pull request #17)
  Fix/ee sensor parameters
  Approved-by: Johannes Pankert <johannes@pankert.eu>
  Approved-by: Martin Wermelinger <martiwer@mavt.ethz.ch>
* Merged in fix/base_sensor_with_2_adapter_plates (pull request #18)
  RKL100 parameters fix
  Approved-by: Martin Wermelinger <martiwer@mavt.ethz.ch>
  Approved-by: Johannes Pankert <johannes@pankert.eu>
* add 2nd adapter plate
* Merge remote-tracking branch 'origin/feature/dario_rafael_master_v2' into fix/ee_sensor_parameters
  # Conflicts:
  #	rokubimini_description/urdf_src/include/R212_parameters.urdf.xacro
* .
* 0.035->0.034
* COM sign
* Merge remote-tracking branch 'origin/fix/urdf_base_sensor' into feature/dario_rafael_master_v2
* Merge remote-tracking branch 'origin/master' into feature/dario_rafael_master_v2
  # Conflicts:
  #	rokubimini_description/urdf_src/include/R212_parameters.urdf.xacro
  #	rokubimini_description/urdf_src/roku_force_torque_sensor.urdf.xacro
  #	rokubimini_gazebo_plugin/config/default.yaml
  #	rokubimini_gazebo_plugin/include/rokubimini_gazebo_plugin/RokubiminiGazeboPlugin.h
  #	rokubimini_gazebo_plugin/src/RokubiminiGazeboPlugin.cpp
* collision model on
* collision off
* base frame position, collision models
* Merged in fix/r212_sensor_frame (pull request #15)
  changed the z offsets of the R212 sensor again to measured values
* changed the z offsets of the R212 sensor again to measured values
* Merged in fix/r212_sensor_frame (pull request #14)
  adapted the sensor frame such that it matches the actual dimensions
  Approved-by: Markus Stäuble <markus.staeuble@mavt.ethz.ch>
* Merge branch 'feature/ft_dario_testing' into feature/dario_rafael_master_v2
* adapted the sensor frame such that it matches the actual dimensions
* Merged in feature/base_sensor_urdf (pull request #12)
  Integration of Base F/T Sensor
  Approved-by: Johannes Pankert <johannes@pankert.eu>
  Approved-by: Martin Wermelinger <martiwer@mavt.ethz.ch>
* split force torque xacro
* shift sensor frame to the correct position
* allow for different rokubimini types
* publish joint state of 'tool'
* Merge remote-tracking branch 'origin/feature/different_ee_sensor_rotation' into feature/dario_rafael_master_v2
* different approach on ee sensor rotation issue
* Merge branch 'feature/ft_dario_testing' into feature/dario_rafael_master_v2
* Merge branch 'feature/ft_dario_testing' of bitbucket.org:leggedrobotics/rokubimini_ethercat_sdk into feature/ft_dario_testing
* turn wrist sensor 90 degrees to align it with real sensor axes
* updated sensor mass from weighing
* change name of topics to {link}_forcetorquesensor}
* apply all changes from force_torque_controllers without ethercat changes
* Merged in feature/urdf (pull request #7)
  Feature/urdf
  Approved-by: Markus Stäuble <markus.staeuble@mavt.ethz.ch>
  Approved-by: Martin Wermelinger <martiwer@mavt.ethz.ch>
* Cleanup mesh.
* Use new version of force sensor
* Use revolute sensor joint only simulation.
* Fix dependencies.
* use PACKAGE_DEPENDENCIES also for catkin_package
* implement comments from PR
* create separate pkg for gazebo plugin and make it independend of any mabi pkg
* Move gazebo plugin xacro here and make joint_states topic adjustable via an argument in the xacro
* move cosmo publisher of readings and jiont status of the force sensor into this repository.
* This configutation works, maybe some tweaking of the friction parameters is needed
* Contributors: Ilias Patsiaouras,Mike Karamousadakis
