# Running the unit tests using pytest

## In one terminal:

```
ros2 run realsense2_ros_mqtt_bridge realsense2_ros_mqtt_bridge
```
## In another terminal:
```
pytest-3 -s --log-cli-level=DEBUG realsense2_ros_mqtt_bridge/test/unittests
```

# Running the system tests using pytest

```
pytest-3 -s --log-cli-level=DEBUG realsense2_ros_mqtt_bridge/test/systemtests
```

Note: To use the log level, user may have to upgrade the pytest to the latest, with the following command

```
python -m pip install -U pytest
```
