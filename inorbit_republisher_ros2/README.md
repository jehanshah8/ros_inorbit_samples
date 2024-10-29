# InOrbit ROS2 Republisher

A ROS2 node that republishes topics based on YAML configuration. This allows bridging between robot-specific topics and InOrbit's expected topic format, as well as bidirectional communication between InOrbit and your robot.

## Installation

### Prerequisites
- ROS2 (Tested with Humble)
- Python 3.8+
- Your robot's ROS2 message types (e.g., `unitree_go_msgs` for Unitree Go2)

### Build Instructions

1. Create a ROS2 workspace (skip if you already have one):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone this repository:
   ```bash
   git clone <repository_url>
   ```

3. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the package:
   ```bash
   colcon build --packages-select inorbit_republisher_ros2
   source install/setup.bash
   ```

## Usage

### Basic Example
Run the republisher with the example config:
```bash
ros2 launch inorbit_republisher_ros2 republisher.launch.py config_file:=$(ros2 pkg prefix inorbit_republisher_ros2)/share/inorbit_republisher_ros2/config/example.yaml
```

### Configuration

The republisher uses YAML configuration files to define topic mappings. Example configurations can be found in the `config` directory:

Run the republisher with an example config:
```bash
ros2 launch inorbit_republisher_ros2 republisher.launch.py config_file:=$(ros2 pkg prefix inorbit_republisher_ros2)/share/inorbit_republisher_ros2/config/example.yaml
```

### Testing Your Configuration

1. Start the republisher with your config:
   ```bash
   ros2 launch inorbit_republisher_ros2 republisher.launch.py config_file:=path/to/your/config.yaml
   ```

2. Monitor the output topics:
   ```bash
   ros2 topic echo /robot_pose  # Or any output topic from your config
   ```

3. Publish test messages:
   ```bash
   ros2 topic pub /robot_state unitree_go_msgs/msg/RobotState "{position: {x: 1.0, y: 2.0, z: 0.0}}"
   ```

## Configuration Reference

### Mapping Types and Options

#### 1. single_field
Maps a single field from the input message to the output.
```yaml
mappings:
  - field: "battery_level"
    mapping_type: "single_field"
    out:
      topic: "/inorbit/custom_data"
      key: "battery"
```

#### 2. json_of_fields
Converts multiple fields into a JSON object.
```yaml
mappings:
  - mapping_type: "json_of_fields"
    field: "position"
    out:
      topic: "/inorbit/custom_data"
      key: "robot_position"
    mapping_options:
      fields: ["x", "y", "z"]  # Specific fields to include
      skip_fields: ["w"]       # Fields to exclude
```

#### 3. array_of_fields
Maps array fields with options for processing.
```yaml
mappings:
  - mapping_type: "array_of_fields"
    field: "joint_states"
    out:
      topic: "/inorbit/custom_data"
      key: "joints"
    mapping_options:
      max_length: 10          # Maximum array length
      start_index: 0          # Starting index
      end_index: 5            # Ending index
      stride: 1               # Step size
```

#### 4. serialize
Serializes entire messages or specified fields.
```yaml
mappings:
  - mapping_type: "serialize"
    out:
      topic: "/robot_control/cmd_vel"
      key: "velocity"
    mapping_options:
      fields: ["linear", "angular"]  # Optional: specific fields to serialize
```

### Advanced Mapping Options

#### Field Selection
```yaml
mapping_options:
  fields: ["x", "y", "z"]     # Include only these fields
  skip_fields: ["timestamp"]   # Exclude these fields
  prefix: "robot_"            # Add prefix to field names
  suffix: "_value"            # Add suffix to field names
```

#### Array Processing
```yaml
mapping_options:
  max_length: 100            # Maximum array length
  start_index: 0             # Starting index
  end_index: -1              # Ending index (-1 for last)
  stride: 2                  # Take every nth element
  reverse: true              # Reverse array order
```

#### Value Transformation
```yaml
mapping_options:
  scale: 1.0                # Multiply numeric values
  offset: 0.0               # Add to numeric values
  precision: 2              # Decimal places for floats
  format: "{:.2f}"          # Python format string
```

### Static Publishers
You can configure static values to be published:
```yaml
static_publishers:
  - out:
      topic: "/inorbit/custom_data"
      key: "robot_version"
    value: "1.0.0"
```

### Value Sources
Static values can come from different sources:
```yaml
static_publishers:
  # From package version
  - out:
      topic: "/inorbit/custom_data"
      key: "robot_version"
    value_from:
      package_version: "my_robot_package"
  
  # From environment variable
  - out:
      topic: "/inorbit/custom_data"
      key: "environment"
    value_from:
      environment_variable: "ROBOT_ENV"
```

### Configuration Management

#### Option 1: Local Config (Recommended for Development)
1. Create a config directory in your workspace:
   ```bash
   mkdir -p ~/ros2_ws/config
   ```

2. Create your config file:
   ```bash
   # Create your custom config
   nano ~/ros2_ws/config/my_robot_config.yaml
   ```

3. Run the republisher with your local config:
   ```bash
   ros2 launch inorbit_republisher_ros2 republisher.launch.py config_file:=$HOME/ros2_ws/config/my_robot_config.yaml
   ```

#### Option 2: System-wide Config (Recommended for Deployment)
1. Create a system config directory:
   ```bash
   sudo mkdir -p /etc/inorbit/config
   sudo chown $USER:$USER /etc/inorbit/config
   ```

2. Create your config file:
   ```bash
   nano /etc/inorbit/config/robot_config.yaml
   ```

3. Run the republisher:
   ```bash
   ros2 launch inorbit_republisher_ros2 republisher.launch.py config_file:=/etc/inorbit/config/robot_config.yaml
   ```

#### Option 3: Package Config (For Distribution)
1. Add your config to the package:
   ```bash
   mkdir -p ~/ros2_ws/src/inorbit_republisher_ros2/config/robots
   cp your_config.yaml ~/ros2_ws/src/inorbit_republisher_ros2/config/robots/
   ```

2. Update setup.py to include your config:
   ```python
   data_files=[
       # ... other entries ...
       (os.path.join('share', package_name, 'config', 'robots'), 
        glob('config/robots/*.yaml')),
   ]
   ```

3. Rebuild and run:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select inorbit_republisher_ros2
   source install/setup.bash
   ros2 launch inorbit_republisher_ros2 republisher.launch.py config_file:=$(ros2 pkg prefix inorbit_republisher_ros2)/share/inorbit_republisher_ros2/config/robots/your_config.yaml
   ```

#### Autostart Configuration (For Production)
1. Create a systemd service:
   ```bash
   sudo nano /etc/systemd/system/inorbit-republisher.service
   ```

2. Add the service configuration:
   ```ini
   [Unit]
   Description=InOrbit ROS2 Republisher
   After=network.target

   [Service]
   Type=simple
   User=robot
   Environment=ROS_DOMAIN_ID=<your_domain>
   ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/robot/ros2_ws/install/setup.bash && ros2 launch inorbit_republisher_ros2 republisher.launch.py config_file:=/etc/inorbit/config/robot_config.yaml'
   Restart=on-failure

   [Install]
   WantedBy=multi-user.target
   ```

3. Enable and start the service:
   ```bash
   sudo systemctl enable inorbit-republisher
   sudo systemctl start inorbit-republisher
   ```

## Troubleshooting

### Common Issues
1. **Topics not being republished**
   - Verify message types match exactly
   - Check that field names exist in the input message
   - Ensure you have sourced your workspace: `source install/setup.bash`

2. **Message type errors**
   - Make sure all required message packages are installed
   - For Unitree: `sudo apt install ros-humble-unitree-go-msgs`

3. **Config file not found**
   - Use absolute paths or the `ros2 pkg prefix` command
   - Verify the config file is installed with the package

## License

MIT License - see the [LICENSE](LICENSE) file for details