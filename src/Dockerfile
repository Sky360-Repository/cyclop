FROM ros:humble-ros-base

# install dependencies
RUN apt-get update && apt-get install -y \
    libqhyccd-dev

# create a catkin workspace
RUN mkdir -p /opt/ros/humble/ws/src

# copy your package to the workspace
COPY qhy_camera_node /opt/ros/humble/ws/src

# build the package
RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install

# source the setup file
ENV  ROS_DISTRO humble
ENV  ROS_WS /opt/ros/humble/ws
ENV  ROS_PACKAGE_PATH /opt/ros/humble/ws:$ROS_PACKAGE_PATH
ENV  PATH $ROS_WS/install/bin:$PATH

# run the node
CMD . /opt/ros/humble/ws/install/setup.sh && ros2 run qhy_camera_node qhy_camera_node
