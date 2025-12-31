^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package realsense2_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.57.3 (2025-09-15)
-------------------
* PR `#3429 <https://github.com/realsenseai/realsense-ros/issues/3429>`_ from remibettan: intel removed, realsense added
* PR `#3417 <https://github.com/realsenseai/realsense-ros/issues/3417>`_ from remibettan: Merging ros2 hkr to ros2 dev final
* PR `#45 <https://github.com/realsenseai/realsense-ros/issues/45>`_ from remibettan: Merge ros2 dev to ros2 hkr
* PR `#3404 <https://github.com/realsenseai/realsense-ros/issues/3404>`_ from adamwhats: Use package:// to discover meshes
* PR `#3410 <https://github.com/realsenseai/realsense-ros/issues/3410>`_ from Nir-Az: Update copyrights
* PR `#3401 <https://github.com/realsenseai/realsense-ros/issues/3401>`_ from ashrafk93: remove test dependencies
* PR `#2813 <https://github.com/realsenseai/realsense-ros/issues/2813>`_ from SamerKhshiboun: Fix URDF and LPCL for SC
* PR `#2807 <https://github.com/realsenseai/realsense-ros/issues/2807>`_ from SamerKhshiboun: Update _d585.urdf.xacro
* PR `#2771 <https://github.com/realsenseai/realsense-ros/issues/2771>`_ from SamerKhshiboun: SC urdf
* Contributors: Arun-Prasad-V, Ashraf Kattoura, Nir Azkiel, Remi Bettan, Samer Khshiboun, adamwhats

4.55.1 (2024-05-28)
-------------------
* PR `#2957 <https://github.com/realsenseai/realsense-ros/issues/2957>`_ from hellototoro: to_urdf fun retrun a str, not a BufferedRandom
* PR `#2953 <https://github.com/realsenseai/realsense-ros/issues/2953>`_ from Arun-Prasad-V: Added urdf & mesh files for D405 model
* PR `#2841 <https://github.com/realsenseai/realsense-ros/issues/2841>`_ from SamerKhshiboun: Remove Dashing, Eloquent, Foxy, L500 and SR300 support
* PR `#2817 <https://github.com/realsenseai/realsense-ros/issues/2817>`_ from karina-ranadive: Replaced Deprecated function mktemp to TemporaryFile
* Contributors: Arun-Prasad-V, karina-ranadive, SamerKhshiboun, hellototoro

4.54.1 (2023-06-27)
-------------------
* Update mesh path
* clone PR1637 to ros2-development
* Fix Apache License Header and Intel Copyrights
* apply copyrights and license on project
* Replace deprecated parameter node_name with name
* Contributors: Arun Prasad, Nir Azkiel, SamerKhshiboun, augustelalande, marqrazz

4.51.1 (2022-09-13)
-------------------
* Add copyright and license to all ROS2-beta source files

* Contributors: SamerKhshiboun

4.0.4 (2022-03-20)
------------------

4.0.3 (2022-03-16)
------------------

4.0.2 (2022-02-24)
------------------

4.0.1 (2022-02-01)
------------------
* Add D455 urdf files

* Contributors: nomumu, JamesChooWK, doronhi

3.1.3 (2020-12-28)
------------------
* fix realsense2_description's dependency to realsense2_camera_msgs
  remove boost dependency.
  rename node_namespace to namespace
  rename node_executable to executable
* Contributors: benlev, Gilaadb, doronhi

3.1.2 (2020-12-10)
------------------
* Add urdf for L515.
* remove librealsense2 and realsense2_camera dependencies
* Add models for D415, D435, D435i.
  For visualization, can be presented using view_model.launch.py
* fix view_d435_model.launch.py and view_d435i_model.launch.py
  run: ros2 launch realsense2_description view_d435i_model.launch.py
* Contributors: Ryan Shim, doronhi

2.2.14 (2020-06-18)
-------------------
* fix urdf issues (arg use_nominal_extrinsics).
* Add noetic support: 
  - urdf files.
  - change state_publisher into robot_state_publisher
* correct offset between camera_link and base_link
* Contributors: Brice, Marco Camurri, doronhi

* upgrade version to 2.2.13
