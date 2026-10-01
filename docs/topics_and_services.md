# Topics and Services

The table below lists all driver-level and per-gripper endpoints and their lifecycle availability.

#### Notes:
- Per-gripper endpoints are namespaced under the driver using the gripper identifier: `/schunk/driver/<gripper_id>/...`
- Service names are consistent across gripper types; however, some underlying service types differ because certain gripper types support different execution modes.
- In the `unconfigured` state, only driver-level endpoints are available; per-gripper topics/services are exposed in `active`.
- Lifecycle transitions follow standard ROS 2 conventions; driver-level lifecycle services are available in multiple states.


## Provided Topics and Services:

| Scope        | Name                                                           | Kind     | Type                                                     | Lifecycle            |
|--------------|----------------------------------------------------------------|----------|----------------------------------------------------------|-----------------------|
| Driver       | /schunk/driver/connection_state                                | Topic    | schunk_gripper_interfaces/msg/ConnectionState            | unconfigured, active  |
| Driver       | /schunk/driver/transition_event                                | Topic    | lifecycle_msgs/msg/TransitionEvent                       | unconfigured, active  |
| Driver       | /parameter_events                                              | Topic    | rcl_interfaces/msg/ParameterEvent                        | unconfigured, active  |
| Driver       | /rosout                                                        | Topic    | rcl_interfaces/msg/Log                                   | unconfigured, active  |
| Driver       | /schunk/driver/scan                                            | Service  | schunk_gripper_interfaces/srv/ScanGrippers               | unconfigured          |
| Driver       | /schunk/driver/add_gripper                                     | Service  | schunk_gripper_interfaces/srv/AddGripper                 | unconfigured          |
| Driver       | /schunk/driver/show_configuration                              | Service  | schunk_gripper_interfaces/srv/ShowConfiguration          | unconfigured          |
| Driver       | /schunk/driver/save_configuration                              | Service  | std_srvs/srv/Trigger                                     | unconfigured, active  |
| Driver       | /schunk/driver/load_previous_configuration                     | Service  | std_srvs/srv/Trigger                                     | unconfigured          |
| Driver       | /schunk/driver/locate_gripper                                  | Service  | schunk_gripper_interfaces/srv/LocateGripper              | unconfigured          |
| Driver       | /schunk/driver/reset_grippers                                  | Service  | std_srvs/srv/Trigger                                     | unconfigured          |
| Driver       | /schunk/driver/list_grippers                                   | Service  | schunk_gripper_interfaces/srv/ListGrippers               | active                |
| Driver       | /schunk/driver/get_state                                       | Service  | lifecycle_msgs/srv/GetState                              | unconfigured, active  |
| Driver       | /schunk/driver/get_available_states                            | Service  | lifecycle_msgs/srv/GetAvailableStates                    | unconfigured, active  |
| Driver       | /schunk/driver/get_available_transitions                       | Service  | lifecycle_msgs/srv/GetAvailableTransitions               | unconfigured, active  |
| Driver       | /schunk/driver/get_transition_graph                            | Service  | lifecycle_msgs/srv/GetAvailableTransitions               | unconfigured, active  |
| Driver       | /schunk/driver/change_state                                    | Service  | lifecycle_msgs/srv/ChangeState                           | unconfigured, active  |
| Driver       | /schunk/driver/describe_parameters                             | Service  | rcl_interfaces/srv/DescribeParameters                    | unconfigured, active  |
| Driver       | /schunk/driver/get_parameter_types                              | Service  | rcl_interfaces/srv/GetParameterTypes                     | unconfigured, active  |
| Driver       | /schunk/driver/get_parameters                                  | Service  | rcl_interfaces/srv/GetParameters                         | unconfigured, active  |
| Driver       | /schunk/driver/list_parameters                                 | Service  | rcl_interfaces/srv/ListParameters                        | unconfigured, active  |
| Driver       | /schunk/driver/set_parameters                                  | Service  | rcl_interfaces/srv/SetParameters                         | unconfigured, active  |
| Driver       | /schunk/driver/set_parameters_atomically                       | Service  | rcl_interfaces/srv/SetParametersAtomically               | unconfigured, active  |
| Per-gripper  | /schunk/driver/<gripper_id>/gripper_state                      | Topic    | schunk_gripper_interfaces/msg/GripperState               | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/joint_states                       | Topic    | sensor_msgs/msg/JointState                               | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/stream/target_position              | Topic    | std_msgs/msg/Float32                                      | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/acknowledge                        | Service  | std_srvs/srv/Trigger                                     | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/brake_test                         | Service  | std_srvs/srv/Trigger                                     | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/fast_stop                          | Service  | std_srvs/srv/Trigger                                     | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/grip                               | Service  | schunk_gripper_interfaces/srv/Grip _(EGU/EZU)_<br>schunk_gripper_interfaces/srv/GripWithGPE _(EGU/EZU+GPE)_<br>schunk_gripper_interfaces/srv/GripWithVelocity _(EGK)_<br>schunk_gripper_interfaces/srv/GripWithVelocityAndGPE _(EGK+GPE)_| active                |
| Per-gripper  | /schunk/driver/<gripper_id>/grip_at_position                   | Service  | schunk_gripper_interfaces/srv/GripAtPosition _(EGU)_<br>schunk_gripper_interfaces/srv/GripAtPositionWithGPE _(EGU+GPE)_<br>schunk_gripper_interfaces/srv/GripAtPositionWithVelocity _(EGK)_<br>schunk_gripper_interfaces/srv/GripAtPositionWithVelocityAndGPE _(EGK+GPE)_      | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/move_to_absolute_position          | Service  | schunk_gripper_interfaces/srv/MoveToAbsolutePosition _(no GPE)_<br>schunk_gripper_interfaces/srv/MoveToAbsolutePositionGPE _(GPE)_  | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/move_to_relative_position          | Service  | schunk_gripper_interfaces/srv/MoveToRelativePosition _(no GPE)_<br>schunk_gripper_interfaces/srv/MoveToRelativePositionGPE _(GPE)_  | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/prepare_for_shutdown               | Service  | std_srvs/srv/Trigger                                     | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/read_parameter                     | Service  | schunk_gripper_interfaces/srv/ReadGripperParameter       | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/release                            | Service  | schunk_gripper_interfaces/srv/ReleaseWithGPE             | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/release_for_manual_movement        | Service  | std_srvs/srv/Trigger                                     | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/show_specification                 | Service  | schunk_gripper_interfaces/srv/ShowGripperSpecification   | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/soft_reset                         | Service  | std_srvs/srv/Trigger                                     | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/start_jogging                      | Service  | schunk_gripper_interfaces/srv/StartJoggingGPE            | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/set_stream                        | Service  | std_srvs/srv/SetBool                                     | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/stop                               | Service  | schunk_gripper_interfaces/srv/StopWithGPE                | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/stop_jogging                       | Service  | std_srvs/srv/Trigger                                     | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/is_stream_enabled                  | Service  | schunk_gripper_interfaces/srv/IsStreamEnabled            | active                |
| Per-gripper  | /schunk/driver/<gripper_id>/write_parameter                    | Service  | schunk_gripper_interfaces/srv/WriteGripperParameter      | active                |
