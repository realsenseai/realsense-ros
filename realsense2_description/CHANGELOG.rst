^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.2 (2021-11-15)
------------------
* Add D455 description
* Add missing aluminum material to d415 model.
* Contributors: Gilad Bretter, doronhi

2.3.1 (2021-07-01)
------------------
* add imu frames to _l515.urdf.xacro
* Contributors: Simon Honigmann, doronhi

2.3.0 (2021-05-05)
------------------

2.2.24 (2021-04-21)
-------------------
* Add conditional param use_mesh.
* Contributors: Teo Cardoso

2.2.23 (2021-03-24)
-------------------

2.2.22 (2021-02-18)
-------------------
* Fix mass of d415
* Consistent add_plug in xacros and launch files
* Contributors: Manuel Stahl, Tim Übelhör, doronhi

2.2.21 (2020-12-31)
-------------------

2.2.20 (2020-11-19)
-------------------
* Add urdf file for l515
* Contributors: doronhi

2.2.18 (2020-10-26)
-------------------
* Feature: Add name to usb_plug to enable multiple copies of usb_plug model.
* Fix mass of d435 - from lgulich
* Contributors: Guillaume, doronhi, lgulich

2.2.17 (2020-09-09)
-------------------

2.2.16 (2020-08-06)
-------------------

2.2.15 (2020-07-13)
-------------------
* Merge remote-tracking branch 'origin/development' into development
* Merge pull request `#1126 <https://github.com/intel-ros/realsense/issues/1126>`_ from mcamurri/add-d435i-xacro
  add D435i modules, urdf and launchfile
* comply to ROS Noetic xacro rules (backcompatible with ROS Melodic)
* Merge branch 'development' into add-d435i-xacro
* Merge branch 'development' of https://github.com/IntelRealSense/realsense-ros into development
  realsense2_description/urdf/test_d435_multiple_cameras.urdf.xacro
* move xml start line in the right place
* add D435i modules, urdf and launchfile
* fix use_nominal_extrinsics arg/property
* add D435i modules, urdf and launchfile
* Contributors: Marco Camurri, doronhi

2.2.14 (2020-06-18)
-------------------
* fix urdf issues (arg use_nominal_extrinsics).
* Add noetic support: 
  - urdf files.
  - change state_publisher into robot_state_publisher
* correct offset between camera_link and base_link
* Contributors: Brice, Marco Camurri, doronhi

* upgrade version to 2.2.13
