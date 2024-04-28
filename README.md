# C++ Example: ROS 2 Bridge

This is an example that shows how to use Rerun's C++ API to log and visualize [ROS 2](https://docs.ros.org/en/humble/index.html) messages (for the ROS 1 version, see [here](https://github.com/rerun-io/cpp-example-ros-bridge)). 

It works by subscribing to all topics with supported types, converting the messages, and logging the data to Rerun. It further allows to remap topic names to specific entity paths, specify additional timeless transforms, and pinhole parameters via an external config file. See the [launch](https://github.com/rerun-io/cpp-example-ros2-bridge/tree/main/rerun_bridge/launch) directory for usage examples.

https://github.com/rerun-io/cpp-example-ros2-bridge/assets/9785832/bbb4675a-b606-4ff4-9e23-ded487020853

This example is built for ROS 2. For more ROS examples, also check out the [ROS 2 example](https://www.rerun.io/docs/howto/ros2-nav-turtlebot), the [URDF data-loader](https://github.com/rerun-io/rerun-loader-python-example-urdf), and the [ROS 1 bridge](https://github.com/rerun-io/cpp-example-ros-bridge).

> NOTE: Currently only `geometry_msgs/{PoseStamped,TransformStamped}`, `nav_msgs/Odometry`,  `tf2_msgs/TFMessage`, and `sensor_msgs/{Image,CameraInfo,Imu}` are supported. However, extending to other messages should be straightforward.

## Compile and run using pixi
The easiest way to get started is to install [pixi](https://prefix.dev/docs/pixi/overview).

The pixi environment described in `pixi.toml` contains all required dependencies, including the example data, and the Rerun viewer. To run the [CARLA](https://carla.org/) example use

```shell
pixi run carla_example
```

## Compile and run using existing ROS environment
If you have an existing ROS workspace and would like to add the Rerun node to it, clone this repository into the workspace's `src` directory and build the workspace.

## Development
Prior to opening a pull request, run `pixi run lint-typos && pixi run cpp-fmt` to check for typos and format the C++ code.

You can update this repository with the latest changes from the [template](https://github.com/rerun-io/rerun_template/) by running:
* `scripts/template_update.py update --languages cpp`
