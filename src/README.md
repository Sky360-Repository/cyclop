# QHY Camera Node

This package contains a ROS2 node for connecting to a QHY 183 camera and grabbing frames. The frames are published to a sensor_msgs/Image topic.

## Dependencies
- QHYCCD (QHY Camera SDK)
- ROS2 (Dashing Diademata or later)
- A QHY 183 camera

## Building the package

```bash
# navigate to your workspace directory
cd <path/to/your/workspace>

# build the package
colcon build --symlink-install

# if you want to build only the specific package
colcon build --symlink-install --packages-select qhy_camera_node
```
This will build all the packages in your workspace or the specific package if you added the --packages-select option. The --symlink-install option creates symlink to the built packages in the install folder so that you don't have to keep re-building everytime you make a change.

You should also make sure that you have all the dependencies of your package installed before building it.

```bash
# To check the dependencies that you need to build your package
colcon list --packages-up-to qhy_camera_node

# To install the dependencies
sudo apt install <dependency1> <dependency2> ...

```

## Running the node

```bash
    source install/setup.bash
    ros2 run qhy_camera_node qhy_camera_node
```

## Parameters

The following parameters can be set in the launch file or on the command line:

* frame_rate (int, default: 30) - the number of frames per second to grab
* exposure_time (float, default: 0.05) - the exposure time in seconds
* binning (int, default: 1) - the binning mode, 1 or 2
* gain (int, default: 0) - the gain, 0-255
* offset (int, default: 0) - the offset, 0-4095

## Topics

camera/image_raw (sensor_msgs/Image) - the raw image data

## Diagnostics

You can use the built-in ROS2 diagnostics to check if the camera is connected and if it is working properly.