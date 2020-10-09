# MoveIt 2 + micro-ROS demo

TODO: demo explanation

## Usage

### micro-ROS

Install and run micro-ROS attitude_estimator demo for ST Discovery board and Zephyr RTOS. 
For detailed info about the micro-ROS build system visit [micro-ROS tutorials](https://micro-ros.github.io/docs/tutorials/core/first_application_linux/)

```bash
# Create a micro-ROS Agent 
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

# Build and flash the micro-ROS app
ros2 run micro_ros_setup create_firmware_ws.sh zephyr discovery_l475_iot1
ros2 run micro_ros_setup configure_firmware.sh attitude_estimator -t serial -d 1
ros2 run micro_ros_setup build_firmware.sh

# Connect the ST Discovery board with the ST-Link USB port and switch the power selector to the correct position
ros2 run micro_ros_setup flash_firmware.sh

# Run the micro-ROS Agent
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev [ST Disco serial device] -v6

```

### MoveIt2

Using a ROS 2 Foxy installation install MoveIt2 as explained in [their webpage](https://moveit.ros.org/install-moveit2/source/).

Now in the same workspace:

```bash
git clone https://github.com/micro-ROS/micro-ROS_moveit2_demo
colcon build --event-handlers desktop_notification- status- --packages-select microros_moveit2_demo --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash

ros2 launch microros_moveit2_demo microros_moveit2_demo.launch.py
```

