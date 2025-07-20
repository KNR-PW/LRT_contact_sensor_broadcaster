# contact_sensors_broadcaster

## Introduction

This project provides a `ros2_control` `ControllerInterface` used to publish [`contact_msgs/Contact.msg`](https://github.com/BartlomiejK2/contact_msgs/blob/main/msg/Contact.msg) messages for every contact sensor, using one state interface for every sensor, where interface values are defined as:

`0.0` - no contact between contact sensor and unknown object (false)

`1.0` - contact between contact sensor and unknown object (true)

For example check out our fork of [gz_ros2_control](https://github.com/KNR-PW/LRT_gz_ros2_control/tree/contact_sensor).

Broadcaster publishes messages on topics defined as:

```
'contact_sensor_frame_id'/contact
```

## ROS 2 version
- Humble 

## Dependencies (all for Humble)
- [`ros2_control`](https://github.com/ros-controls/ros2_control)
- [`ros2_controllers`](https://github.com/ros-controls/ros2_controllers)
- [`generate_parameter_library`](https://github.com/PickNikRobotics/generate_parameter_library)
- [`contact_msgs`](https://github.com/BartlomiejK2/contact_msgs)

## Installation
1. Install `ros2_control` for Humble (if you have not already)
  
2. Clone `contact_msgs` repo to your workspace:
```bash
git clone https://github.com/BartlomiejK2/contact_msgs.git
```
3. Clone this repo to your workspace:
```bash
git clone https://github.com/KNR-PW/LRT_contact_sensor_broadcaster.git
```
4. Install dependencies in workspace:
```bash
rosdep install --ignore-src --from-paths . -y -r
```
5. Build:
```bash
colcon b
```

## Controller parameters (example):

```yaml
...
contact_sensors_broadcaster:
      type: contact_sensors_broadcaster/ContactSensorsBroadcaster
...
contact_sensors_broadcaster:
  ros__parameters:
    frame_ids:
    - contact_sensor_1
    - contact_sensor_2
    sensor_names:
    - arm_contact_sensor
    - foot_contact_sensor
...
```

#### `frame_ids` - name of frames, where contact sensor is defined
#### `sensor_names` - name of contact sensors 

### :warning: IMPORTANT: SENSOR NAMES HAVE TO BE SAME AS IN `ros2_control` TAG AND `gazebo reference` TAG FOR THIS SENSORS ([example](https://github.com/KNR-PW/LRT_gz_ros2_control/blob/contact_sensor/gz_ros2_control_demos/urdf/test_contact_sensor.xacro.urdf))

## Troubleshooting/New functionality
#### Add new `Issue`. I will try my best to answer. You can also contribute to project via pull requests.

## Contributing
1. Change code.
2. Run all launchfile tests. 
3. Check performance, compliance and other functionality via e.g. [plotjugller](https://plotjuggler.io/) for `ros2`.
4. Add clear description what changes you've made in pull request.
