#!/usr/bin/env python3

import rclpy
from inorbit_republisher_ros2.republisher import InOrbitRepublisher
import sys
import os

def main(args=None):
    rclpy.init(args=args)
    node = InOrbitRepublisher()
    
    # Try getting config from environment variable
    config_file = os.environ.get('INORBIT_CONFIG_FILE')
    
    # If not in environment, try ROS parameter
    if not config_file:
        node.declare_parameter('config_file', '')
        config_file = node.get_parameter('config_file').value
    
    if config_file:
        try:
            with open(config_file, 'r') as f:
                config = f.read()
        except Exception as e:
            node.get_logger().error(f'Failed to read config file {config_file}: {e}')
            return 1
    else:
        # Try getting config directly from parameter
        node.declare_parameter('config', '')
        config = node.get_parameter('config').value
        if not config:
            node.get_logger().error('No configuration provided via file or parameter')
            return 1
        
    if not node.load_config(config):
        return 1
        
    node.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    sys.exit(main()) 