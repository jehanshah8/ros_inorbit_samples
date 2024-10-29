#!/usr/bin/env python3

# Copyright 2021 InOrbit, Inc.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# 
# ROS to InOrbit republisher node sample
#
# It uses a YAML-based configuration to map between arbitrary
# ROS topics into InOrbit key/value custom data topics.

import rclpy
from rclpy.node import Node
import yaml
from std_msgs.msg import String
from rosidl_runtime_py.utilities import get_message
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from operator import attrgetter
import json
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from rosidl_runtime_py.convert import message_to_ordereddict
import os

# Add these constants at the top of the file
MAPPING_TYPE_SINGLE_FIELD = "single_field"
MAPPING_TYPE_ARRAY_OF_FIELDS = "array_of_fields"
MAPPING_TYPE_JSON_OF_FIELDS = "json_of_fields"
MAPPING_TYPE_SERIALIZE = "serialize"

STATIC_VALUE_FROM_PACKAGE_VERSION = "package_version"
STATIC_VALUE_FROM_ENVIRONMENT_VAR = "environment_variable"

class InOrbitRepublisher(Node):
    def __init__(self):
        super().__init__('inorbit_republisher')
        self.pubs = {}  # Dictionary of publisher instances
        self.subs = {}  # Dictionary of subscriber instances
        
    def load_config(self, config_yaml):
        """Load and validate configuration from yaml"""
        try:
            self.config = yaml.safe_load(config_yaml)
            return True
        except yaml.YAMLError as e:
            self.get_logger().error(f'Failed to load configuration: {e}')
            return False 

    def get_message_class(self, msg_type):
        """Load message class from string"""
        try:
            # Convert 'std_msgs/String' format to module path format
            pkg_name, msg_name = msg_type.split('/')
            # In ROS2, we need to use the fully qualified path
            msg_class = get_message(f'{pkg_name}/msg/{msg_name}')
            return msg_class
        except (ValueError, ImportError) as e:
            self.get_logger().warning(f'Failed to load message type {msg_type}: {e}')
            return None
            
    def create_topic_publisher(self, topic, msg_type, latched=False):
        """Create a publisher for the given topic and message type"""
        msg_class = self.get_message_class(msg_type)
        if msg_class is None:
            return None
            
        qos = QoSProfile(depth=100)
        if latched:
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            
        pub = self.create_publisher(
            msg_class,
            topic,
            qos
        )
        self.pubs[topic] = pub
        return pub

    def extract_value(self, msg, getter_fn):
        """Extracts a value from the given message using the provided getter function"""
        try:
            val = getter_fn(msg)
            return val
        except AttributeError as e:
            self.get_logger().warning(f'Failed to extract value: {e}')
            return None

    def process_single_field(self, field_value, mapping):
        """
        Processes a scalar value before publishing according to mapping options
        If a 'filter' function is provided, returns the value only if it passes the filter
        """
        filter_fn = mapping.get('mapping_options', {}).get('filter')
        return field_value if not filter_fn or eval(filter_fn)(field_value) else None

    def process_array(self, field, mapping):
        """
        Processes a given array field from the ROS message and:
        - Filters it using the 'filter' function (if provided)
        - For each element, gets the set of keys defined by array_fields
        - Returns a JSON string containing the resulting array of objects
        """
        values = {
            'data': []
        }

        # Apply filter if specified
        filter_fn = mapping.get('mapping_options', {}).get('filter')
        filtered_array = field
        if filter_fn:
            filter_function = eval(filter_fn)
            filtered_array = [item for item in field if filter_function(item)]

        # Extract specified fields or use whole array
        if 'fields' in mapping.get('mapping_options', {}):
            fields = mapping['mapping_options']['fields']
            try:
                # Try to extract all fields for one element first to validate
                test_elem = filtered_array[0]
                for f in fields:
                    _ = attrgetter(f)(test_elem)  # Will raise AttributeError if field doesn't exist
                
                # If we get here, all fields exist
                values['data'] = [{f: self.extract_value(elem, attrgetter(f)) 
                                 for f in fields} 
                                for elem in filtered_array]
            except (AttributeError, IndexError) as e:
                self.get_logger().warning(f'Failed to extract array fields: {e}')
                return None
        else:
            values['data'] = filtered_array

        return json.dumps(values)

    def extract_values_as_dict(self, msg, mapping):
        """
        Extracts several values from a given nested msg field and returns a dictionary of
        <field, value> elements
        """
        values = {}
        try:
            base_getter_fn = attrgetter(mapping['field'])
            base_value = base_getter_fn(msg)
            fields = mapping.get('mapping_options', {}).get('fields')
            
            for field in fields:
                getter_fn = attrgetter(field)
                try:
                    val = getter_fn(base_value)
                    # Handle ROS2 Time messages
                    if isinstance(val, TimeMsg):
                        val = float(val.sec) + float(val.nanosec) / 1e9
                    values[field] = val
                except AttributeError as e:
                    self.get_logger().warning(f'Couldn\'t get attribute {field}: {e}')
                    values[field] = None
                    
            # Apply filter if specified
            filter_fn = mapping.get('mapping_options', {}).get('filter')
            if filter_fn and not eval(filter_fn)(values):
                return None
            return values
            
        except AttributeError as e:
            self.get_logger().warning(f'Failed to get base value: {e}')
            return None

    def serialize(self, msg, mapping):
        """
        Transforms the ROS message to json and:
        - Filters out fields on the top level of the json only.
        """
        # Get fields configuration
        fields = mapping.get('mapping_options', {}).get('fields')
        
        # Transform ROS message to dict
        try:
            msg_dict = message_to_ordereddict(msg)
            
            # Convert OrderedDict to regular dict
            msg_dict = dict(msg_dict)
            
            # Shrink output by keeping only selected fields or keys
            if fields:
                msg_dict = {k: v for k, v in msg_dict.items() if k in fields}
                
            return msg_dict
        except Exception as e:
            self.get_logger().warning(f'Failed to serialize message: {e}')
            return None

    def setup_republishers(self):
        """Set up ROS topic republishers based on configuration"""
        republishers = self.config.get('republishers', ())
        for repub in republishers:
            # Load subscriber message type
            msg_class = self.get_message_class(repub['msg_type'])
            if msg_class is None:
                continue

            # Get latched option
            latched = repub.get("latched", False)
            in_topic = repub['topic']

            # Create publisher for each new outgoing topic
            for mapping in repub['mappings']:
                out_topic = mapping['out']['topic']
                # Create unique publisher key for latched topics
                pub_key = f"{out_topic}+{in_topic}" if latched else out_topic
                if pub_key not in self.pubs:
                    self.pubs[pub_key] = self.create_topic_publisher(out_topic, "std_msgs/String", latched)

            # Create subscription with callback
            self.create_subscription(
                msg_class,
                in_topic,
                lambda msg, r=repub, t=in_topic, l=latched: 
                    self.republisher_callback(msg, r, t, l),
                10
            )

    def republisher_callback(self, msg, repub, in_topic, latched):
        """Handle incoming messages and republish according to mappings"""
        for mapping in repub['mappings']:
            key = mapping['out']['key']
            val = None
            mapping_type = mapping.get('mapping_type', MAPPING_TYPE_SINGLE_FIELD)
            topic = mapping['out']['topic']

            try:
                if mapping_type == MAPPING_TYPE_SINGLE_FIELD:
                    field = self.extract_value(msg, attrgetter(mapping['field']))
                    val = self.process_single_field(field, mapping)

                elif mapping_type == MAPPING_TYPE_ARRAY_OF_FIELDS:
                    field = self.extract_value(msg, attrgetter(mapping['field']))
                    val = self.process_array(field, mapping)

                elif mapping_type == MAPPING_TYPE_JSON_OF_FIELDS:
                    val = self.extract_values_as_dict(msg, mapping)
                    if val:
                        val = json.dumps(val)

                elif mapping_type == MAPPING_TYPE_SERIALIZE:
                    val = self.serialize(msg, mapping)
                    if val:
                        val = json.dumps(val)

                if val is not None:
                    pub_key = f"{topic}+{in_topic}" if latched else topic
                    self.pubs[pub_key].publish(String(data=f"{key}={val}"))

            except Exception as e:
                self.get_logger().warning(f'Failed to process mapping: {e}')

    def get_package_version(self, package_name):
        """Get package version from package.xml"""
        try:
            from ament_index_python.packages import get_package_share_directory
            import xml.etree.ElementTree as ET
            
            # Get package share directory
            pkg_share = get_package_share_directory(package_name)
            pkg_xml = os.path.join(pkg_share, 'package.xml')
            
            # Parse package.xml
            tree = ET.parse(pkg_xml)
            root = tree.getroot()
            
            # Find version tag
            version = root.find('version')
            if version is not None:
                return version.text
                
        except Exception as e:
            self.get_logger().warn(f'Failed to get package version: {str(e)}')
        return None

    def setup_static_publishers(self):
        """Set up static publishers based on configuration"""
        for static_pub in self.config.get('static_publishers', ()):
            out = static_pub['out']
            topic = out['topic']
            key = out['key']
            
            # Get value
            value = static_pub.get('value')
            if value is None and 'value_from' in static_pub:
                value_from = static_pub['value_from']
                if 'package_version' in value_from:
                    value = self.get_package_version(value_from['package_version'])
                elif 'environment_variable' in value_from:
                    value = os.environ.get(value_from['environment_variable'])
            
            if value is not None:
                # Create publisher if needed
                if topic not in self.pubs:
                    self.pubs[topic] = self.create_topic_publisher(topic, 'std_msgs/String')
                
                # Publish value
                msg = String()
                msg.data = f"{key}={value}"
                self.pubs[topic].publish(msg)

    def start(self):
        """Initialize and start the republisher"""
        self.get_logger().info('Setting up republishers...')
        self.setup_republishers()
        self.setup_static_publishers()
        self.get_logger().info('Republisher started')