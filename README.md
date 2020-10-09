# MoveIt 2 + micro-ROS demo

TODO: demo explanation

## Usage

Using a ROS 2 Foxy installation install MoveIt2 as explained in [their webpage](https://moveit.ros.org/install-moveit2/source/).

Now in the same workspace:

```bash
git clone [this repo]
colcon build --event-handlers desktop_notification- status- --packages-select microros_moveit2_demo --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash

ros2 launch microros_moveit2_demo microros_moveit2_demo.launch.py
```