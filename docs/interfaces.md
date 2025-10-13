# Controller Interfaces

## ROS 2 Topics (Published)
| Topic | Message Type | Origin | Description |
|-------|--------------|--------|-------------|
| `/controller/audio` | `robot_interfaces/msg/StringTime` | `controller/nodes/audio_node.py` | Requests audio playback on the robot side. |
| `/controller/lights` | `robot_interfaces/msg/StringTime` | `controller/nodes/relays_node.py` | Commands light sequences / relays. |
| `/controller/wheels` | `robot_interfaces/msg/StringTime` | `controller/nodes/wheels_node.py` | Movement commands for the wheeled base. |
| `/controller/system` | `robot_interfaces/msg/StringTime` | `controller/nodes/system_node.py` | Misc system-level commands. |

> The precise list of topics may evolve. Keep this table aligned with the node implementations and the subscriber list in `dadou_robot_ros`.

## Input Devices
- **USB Gamepad**: Normalised button/axis state, mapping defined in `controller/buttons/button_config.py`.
- **RP2040 Glove**: Scans flex sensors/buttons; communicates over serial through `controller/input/serial_inputs.py`.
- **GUI**: Tkinter windows trigger the same commands as physical inputs to facilitate rehearsals.

## Configuration Files
- `controller/control_config.py`: Master configuration referencing playlists, scenes, and button layouts.
- `conf/ros2/`: ROS 2 launch files and environment configuration for controller deployment.
- `json/playlists/`: Playlists/scenes triggered by controller inputs.

## Logs & Telemetry
- Default log directory: `test_logs/` for local execution; on the robot the path is adjusted through shared constants (`dadou_utils_ros.logging_conf`).
- ROS logging uses the shared logging formatter from `dadou_utils_ros/logging_conf.py`, which now embeds the class name for easier grepping.

## External Dependencies
- Relies on `dadou_utils_ros` modules (e.g., `utils_static`, `logging_conf`).
- Uses `pygame` for gamepad support and `colorlog` for coloured logging.

See also the subscriber side in `dadou_robot_ros/docs/interfaces.md` (to be created alongside this file).
