# C++ Example: ROS 2 Bridge

This is an example that shows how to use Rerun's C++ API to log and visualize [ROS 2](https://docs.ros.org/en/humble/index.html) messages (see [ROS 1 bridge](https://github.com/rerun-io/cpp-example-ros-bridge)). 

It works by subscribing to all topics with supported types, converting the messages, and logging the data to Rerun. It further allows to remap topic names to specific entity paths, specify additional timeless transforms, and pinhole parameters via an external config file. See the [launch](https://github.com/rerun-io/cpp-example-ros2-bridge/tree/main/rerun_bridge/launch) directory for usage examples.

TODO(leo) add new video

This example is built for ROS 2. For more ROS examples, also check out the [ROS 2 example](https://www.rerun.io/docs/howto/ros2-nav-turtlebot), the [URDF data-loader](https://github.com/rerun-io/rerun-loader-python-example-urdf), and the [ROS 1 bridge](https://github.com/rerun-io/cpp-example-ros-bridge).

> NOTE: Currently only `geometry_msgs/{PoseStamped,TransformStamped}`, `nav_msgs/Odometry`,  `tf2_msgs/TFMessage`, and `sensor_msgs/{Image,CameraInfo,Imu}` are supported. However, extending to other messages should be straightforward.

## Compile and run using pixi
The easiest way to get started is to install [pixi](https://prefix.dev/docs/pixi/overview).

TODO(leo) instructions to run example

## Compile and run using existing ROS environment
If you have an existing ROS workspace and would like to add the Rerun node to it, clone this repository into the workspace's `src` directory and build the workspace.

## Development
You can update this repository with the latest changes from the template by running:
* `scripts/template_update.py update --languages cpp`

To format the code use `pixi run cpp-fmt`.
