# Examples

These examples demonstrate how to use the SCHUNK Gripper ROS2 driver primarily via CLI commands.
All actions shown here, such as adding grippers, issuing commands, and managing the driver lifecycle, can also be performed programmatically using ROS2 service calls from your own code.

**Important:** Whenever a command fails, the gripper firmware enters an internal error state. In this state, no movement commands are possible until the gripper is acknowledged using the `acknowledge` service. This step is essential to return the gripper to an operational state and is not typical in standard ROS2 workflows, so be sure to include it whenever an error occurs.


## Example 1: Launch empty driver in normal mode

Launch an empty driver in normal mode (`headless:=false`):

```bash
ros2 launch schunk_gripper_driver driver.launch.py
```

The driver starts in the `unconfigured` state and no grippers are connected yet.
You can list all available topics and services:

```bash
ros2 node info /schunk/driver
```

In this example, we add a gripper connected via Modbus:

```bash
ros2 service call /schunk/driver/add_gripper schunk_gripper_interfaces/srv/AddGripper "gripper:
  host: ''
  port: 0
  serial_port: '/dev/ttyUSB0'
  device_id: 13"
```

Verify that the gripper was added to the configuration:

```bash
ros2 service call /schunk/driver/show_configuration schunk_gripper_interfaces/srv/ShowConfiguration
```

Subscribing to the `connection_state` topic shows that added grippers are not connected yet at this lifecycle stage:

```bash
ros2 topic echo /schunk/driver/connection_state
```

You could add more grippers at this point. Next, transition the driver to the `active` lifecycle state:

```bash
ros2 lifecycle set /schunk/driver configure
ros2 lifecycle set /schunk/driver activate
```

Subscribing again to the `connection_state` topic will show that the previously added gripper is now connected.
List all available topics and services:

```bash
ros2 node info /schunk/driver
```

You can see that the list of available services has changed. Most importantly, **the gripper-specific services** are now available.
For example, issue a grip command to the added gripper:

```bash
ros2 service call /schunk/driver/EGU_50_MB_M_B_1/grip schunk_gripper_interfaces/srv/GripWithGPE "force: 100
use_gpe: true
outward: false"
```

> Note: In this example, the gripper is an EGU with GPE functionality. For other gripper types, the grip service type may differ. See the [full list of service types](./topics_and_services.md).

**Important:** Whenever a command fails, the gripper firmware enters an internal error state. In this state, no movement commands are possible anymore. To bring the gripper back into the operational state, use the `acknowledge` service:

```bash
ros2 service call /schunk/driver/EGU_50_MB_M_B_1/acknowledge std_srvs/srv/Trigger
```

To save this gripper configuration so it is automatically loaded in the next launch (see _Example 2_), run:

```bash
ros2 service call /schunk/driver/save_configuration std_srvs/srv/Trigger
```


## Example 2: Launch driver in headless mode

In headless mode, the driver automatically loads all grippers that were previously saved using the `save_configuration` service (as shown in _Example 1_).
The driver immediately transitions to the `active` lifecycle state, and all saved grippers are connected and ready to receive commands.

Launch the driver in headless mode:

```bash
ros2 launch schunk_gripper_driver driver.launch.py headless:=true
```

Verify that the driver is in the `active` state:

```bash
ros2 lifecycle get /schunk/driver
```

Check the connection state of all grippers:

```bash
ros2 topic echo /schunk/driver/connection_state
```

List all available topics and services:

```bash
ros2 node info /schunk/driver
```

> Note: In headless mode, all previously saved grippers are automatically connected and ready to receive commands. You can now directly issue commands such as grip, move, jog, etc. as described in _Example 1_.


## Example 3: Launch driver with explicit gripper connection parameters

Instead of adding grippers manually or using a saved configuration, you can provide the connection parameters directly via launch arguments.
This example connects a Modbus gripper with ID 13 at serial port `/dev/ttyUSB0` in normal mode (`headless:=false`).

Launch the driver:

```bash
ros2 launch schunk_gripper_driver driver.launch.py serial_port:=/dev/ttyUSB0 device_id:=13 baudrate:=115200
```

The driver starts in the `unconfigured` state. You can now proceed as in _Example 1_.

The `baudrate` argument controls the Modbus baudrate. Its default value is `115200`.


## Example 4: Stream target positions

After the driver is active and the gripper is connected, enable target-position streaming:

```bash
ros2 service call /schunk/driver/EGU_50_MB_M_B_1/set_stream std_srvs/srv/SetBool "{data: true}"
```

Publish target positions in meters to the gripper's stream topic:

```bash
ros2 topic pub --rate 20 /schunk/driver/EGU_50_MB_M_B_1/stream/target_position std_msgs/msg/Float32 "{data: 0.05}"
```

While streaming is enabled, regular motion commands are rejected. Stop and shutdown-related commands remain available. Disable streaming before sending regular motion commands again:

```bash
ros2 service call /schunk/driver/EGU_50_MB_M_B_1/set_stream std_srvs/srv/SetBool "{data: false}"
```

You can query the current streaming state with:

```bash
ros2 service call /schunk/driver/EGU_50_MB_M_B_1/is_stream_enabled schunk_gripper_interfaces/srv/IsStreamEnabled
```
