# Testing realsense2_camera
The test infra for realsense2_camera uses both gtest and pytest. gtest is typically used here for testing at the unit level and pytest for integration level testing. Please be aware that the README assumes that the ROS2 version used is Humble or later, as the launch_pytest package used here is not available in prior versions

## Test using gtest
The default folder for the test cpp files is realsense2_camera/test. A test template gtest_template.cpp is available in the same folder.
### Adding a new test
If the user wants to add a new test, a copy of gtest_template.cpp as the starting point. Please name the file as gtest_`testname`.cpp format so that the CMake detects the file and add as a new test. Please be aware that, multiple tests can be added into one file, the gtest_template.cpp has 2 tests, test1 and test2.

### Adding a new test folder
It is recommended to use the test folder itself for storing all the cpp tests. However, if the user wants to add a different folder for a set of tests, please ensure that the file name format mentioned above is followed. The folder path is added to realsense_camera/CMakeLists.txt as below for the build to detect the tests within.

```
  find_package(ament_cmake_gtest REQUIRED)
  set(_gtest_folders
    test                        #<-- default folder for the gtest sources
    new_folder_for_test_but_why #<-- new folder name is added
  )
```

## Test using pytest
The default folder for the test py files is realsense2_camera/test. Two test template files test_launch_template.py and test_integration_template.py are available in the same folder for reference.
### Add a new test
To add a new test, the user can create a copy of the test_launch_template.py or test_integration_template.py and start from there. Please name the file in the format test_`testname`.py so that the CMake detects the file and add as a new test. Please be aware that, multiple tests can be added into one file, the test_integration_template.py itself has more than one test. The marker `@pytest.mark.launch` is used to specify the test entry point.  

The test_launch_template.py uses the rs_launch.py to start the camera node, so this template can be used for testing the rs_launch.py together with the rs node.

The test_integration_template.py gives a better control for testing, it uses few util functions and base test class from pytest_rs_utils.py located at the same location. However, it doesn't use the rs_launch.py, it creates the node directly instead.

The test_integration_template.py has two types of tests, one has a function "test_using_function". If the user wants to have a better control over the launch context for any specific test scenarios, this can be used. Both the function based test and class based tests use a default launch configuration from the utils. It's recommended to modify the camera name to a unique one in the parameters itself so that there are not clashes between tests. 

It is expected that the class based test is used as the test format for most of the usecases. The class based test inherits from `pytest_rs_utils.RsTestBaseClass` and it has three steps, namely: init, run_test and process_data. Unless for the basic tests, the user will have to override the process_data function and check if the data received from the topics are as expected. Also, if the user doesn't want the base class to modify the data, use 'store_raw_data':True in the theme definition. Please see the test_integration_template.py for reference.

An assert command can be used to indicate if the test failed or passed. Please see the template for more info.

### Adding a new test folder
It is recommended to use the test folder itself for storing all the pytests. However, if the user wants to add a different folder for a set of tests, please ensure that the file name format mentioned above is followed. The folder path should be added to realsense_camera/CMakeLists.txt as below for the infra to detect the new test folder and the tests within.

```
find_package(ament_cmake_pytest REQUIRED)
set(_pytest_folders
test                  #default test folder
test/templates
test/rosbag
new_folder_for_pytest #<-- new folder #but please be aware that the utils functions are in test/utils folder, 
                                      #so if the template is used, change the include path also accordingly
)
```

### Grouping of tests  
The pytests can be grouped using markers. These markers can be used to run a group of tests. However, "colcon test" command doesn't pass a custom marker using (--pytest-args -m `marker_name`) to the pytest internally. This is because, the ament_cmake that works as a bridge between colcon and pytest doesn't pass the pytest arguments to pytest. So until this is fixed, pytest command has to be used directly for running a group of tests. Please see the next session for the commands to run a group of py tests.

The grouping is specified by adding a marker just before the test declaration. In the test_integration_template.py `rosbag` is specified as a marker specify tests that use rosbag file. This is achieved by adding "@pytest.mark.rosbag" to the begining of the test. So when the pytest parses for test, it detects the marker for the test. If this marker is selected or none of the markers are specified, the test will be added to the list, else will be listed as a deselected test.

It is recommended to use markers such as d457, rosbag, or d435i etc to differentiate the tests so that it's easier to run a group of tests in a machine that has the required hardware.
 
## Building and running tests  

### Build steps 

The command used for building the tests along with the node:

	colcon build

The test statements in CMakeLists.txt are protected by BUILD_TESTING macro. So in case, the tests are not being built, then it could be that the macro are disabled by default.

Note: The below command helps view the steps taken by the build command.

	colcon build --event-handlers console_direct+

### Prerequisites for running the tests

1. The template tests require the rosbag files from librealsense.intel.comi, the following commands download them:
```
bag_filename="https://librealsense.intel.com/rs-tests/TestData/outdoors_1color.bag";
wget $bag_filename -P "records/"
bag_filename="https://librealsense.intel.com/rs-tests/D435i_Depth_and_IMU_Stands_still.bag";
wget $bag_filename -P "records/"
```
2. The tests use the environment variable ROSBAG_FILE_PATH as the directory that contains the rosbag files
```	
export ROSBAG_FILE_PATH=/path/to/directory/of/rosbag
```
3. Install launch_pytest package. For humble: 
```
sudo apt install ros-$ROS_DISTRO-launch-pytest
```
4. As in the case of all the packages, the install script of realsesnse2_camera has to be run.
```
. install/local_setup.bash
```
5. If the tests are run on a machine that has the RS board connected or the tests are using rosbag files, then its better to let the ROS search for the nodes in the local machine, this will be faster and less prone to interference and hence unexpected errors. It can be achieved using the following environment variable.
```
export ROS_DOMAIN_ID=1
```

So, all put together:

```
sudo apt install  ros-$ROS_DISTRO-launch-pytest
bag_filename="https://librealsense.intel.com/rs-tests/TestData/outdoors_1color.bag";
wget $bag_filename -P "records/"
bag_filename="https://librealsense.intel.com/rs-tests/D435i_Depth_and_IMU_Stands_still.bag";
wget $bag_filename -P "records/"
export ROSBAG_FILE_PATH=$PWD/records
. install/local_setup.bash
export ROS_DOMAIN_ID=1
```

### Running the tests using colcon

All the tests can be run using the below command:

	colcon test --packages-select realsense2_camera

This command will invoke both gtest and pytest infra and run all the tests specified in the files mentioned above. Since the test results are stored in build/realsense2_camera/test_results folder, it's good to clean this up after running the tests with a new test added/removed.

The same command with console_direct can be used for more info on failing tests, as below:

	colcon test --packages-select realsense2_camera --event-handlers console_direct+

The test results can be viewed using the command:

	colcon test-result --all --test-result-base build/realsense2_camera/test_results/

The xml files mentioned by the command can be directly opened also.

### Running pytests directly

Note :
1. All the commands for test execution has to be executed from realsense-ros folder. For example: If the ROS2 workspace was created based on Step 3 [Option2] of [this](https://github.com/IntelRealSense/realsense-ros/blob/ros2-master/README.md#installation).
Then, the path to execute the tests would be ~/ros2_ws/src/realsense-ros.

		cd ~/ros2_ws/src/realsense-ros

2. Please setup below required environment variables. If not, Please prefix them for every test execution.

		export PYTHONPATH=$PYTHONPATH:$PWD/realsense2_camera/test/utils:$PWD/realsense2_camera//launch:$PWD/realsense2_camera//scripts

User can run all the tests in a pytest file directly using the below command:

	pytest-3 -s realsense2_camera/test/test_integration_template.py

All the pytests in a test folder can be directly run using the below command:

	pytest-3 -s realsense2_camera/test/

### Running a group of pytests
As mentioned above, a set of pytests that are grouped using markers can be run using the pytest command. The below command runs all the pytests in realsense2_camera/test folder that has the marker rosbag:

	pytest-3 -s -m rosbag realsense2_camera/test/


### Running a single pytest
The below command finds the test with the name test_static_tf_1 in realsense2_camera/test folder run:

	pytest-3 -s -k test_static_tf_1 realsense2_camera/test/

### Marking tests as regression tests
Some of the tests, especially the live tests with multiple runs, for e.g., all profile tests (test_camera_all_profile_tests.py) take a long time. Such tests are marked are skipped with condition so that "colcon test" skips it.
If a user wants to add a test to this conditional skip, user can add by adding a marker as below.

@pytest.mark.skipif (os.getenv('RS_ROS_REGRESSION', "not found") == "not found",reason="Regression is not enabled, define RS_ROS_REGRESSION")

### Running skipped regression tests
1. Set the environment variable RS_ROS_REGRESSION as 1 and run the "colcon test"
2. pytest example: 
	RS_ROS_REGRESSION=1 PYTHONPATH=$PYTHONPATH:$PWD/realsense2_camera/test/utils:$PWD/realsense2_camera//launch:$PWD/realsense2_camera//scripts pytest-3 -s realsense2_camera/test/live_camera/test_camera_aligned_tests.py -k test_camera_align_depth_color_all -m d415

## Points to be noted while writing pytests
The tests that are in one file are normally run in parallel, there could also be changes in the pytest plugin. So if there are multiple tests in one file, the system capacity can influence the test execution. It's recomended to have 3-4 tests in file, more than that can affect the test results due to delays.
### Passing/changing parameters
The parameters passed while creating the node can be initialized individually for each test, please see the test_parameterized_template example for reference. The default values are taken from rs_launch.py and the passed parameters are used for overriding the default values. The parameters that can be dynamically modified can be changed using the param interface provided. However, the function `create_service_client_ifs()` has to be called to create this interface. Please see the test_d455_basic_tests.py for reference. There are specific functions to change the string, integer and bool parameters, the utils can be extended if any more types are needed.
### Difference in setting the bool parameters
There is a difference between setting the default values of bool parameters and setting them dynamically.
The bool test params are assinged withn quotes as below.
	test_params_all_profiles_d455 = {
		'camera_name': 'D455',
		'device_type': 'D455',
		'enable_accel':"True",
		'enable_gyro':"True",
		'unite_imu_method':1,
		}

However the function that implements the setting of bool parameter dynamically takes the python bool datatype. For example:
	self.set_bool_param('enable_accel', False)
### Adding 'service call' client interface
1. Create a client for respective service call in method `create_service_client_ifs()`, refer client creation for service call `device_info` with service type `DeviceInfo`.
2. Create a `get_` or `set_` method for each service call, refer `get_deviceinfo` in pytest_rs_utils.py
3. Use the `get_`, `set_` service calls as used in test_camera_service_call.py
