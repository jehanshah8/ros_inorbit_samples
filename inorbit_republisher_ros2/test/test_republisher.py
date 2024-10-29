import unittest
import yaml
from inorbit_republisher_ros2.republisher import InOrbitRepublisher
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from operator import attrgetter
import json
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header
import os
from threading import Event
from rclpy.executors import SingleThreadedExecutor
import tempfile
import time

class TestInOrbitRepublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Ignore if already shutdown

    def setUp(self):
        self.node = InOrbitRepublisher()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def test_load_valid_config(self):
        test_config = """
        republishers:
          - topic: "/test_topic"
            msg_type: "std_msgs/String"
            mappings:
              - field: "data"
                out:
                  topic: "/inorbit/custom_data"
                  key: "test_key"
        """
        self.assertTrue(self.node.load_config(test_config))
        self.assertIn('republishers', self.node.config)

    def test_load_invalid_config(self):
        test_config = """
        republishers:
          - topic: "/test_topic"
            msg_type: "std_msgs/String
            mappings:  # Invalid YAML (missing quote)
        """
        self.assertFalse(self.node.load_config(test_config))

    def test_get_message_class(self):
        # Test valid message type
        msg_class = self.node.get_message_class("std_msgs/String")
        self.assertEqual(msg_class, String)
        
        # Test invalid message type
        msg_class = self.node.get_message_class("invalid_msgs/InvalidType")
        self.assertIsNone(msg_class)
        
    def test_create_publisher(self):
        topic = "/test/topic"
        msg_type = "std_msgs/String"
        
        # Test regular publisher creation
        pub = self.node.create_topic_publisher(topic, msg_type)
        self.assertIsNotNone(pub)
        self.assertIn(topic, self.node.pubs)
        
        # Test latched publisher creation
        pub = self.node.create_topic_publisher(topic, msg_type, latched=True)
        self.assertIsNotNone(pub)
        self.assertIn(topic, self.node.pubs)

    def test_extract_and_process_value(self):
        # Test basic value extraction
        test_msg = String(data="test_value")
        getter = attrgetter('data')
        value = self.node.extract_value(test_msg, getter)
        self.assertEqual(value, "test_value")

        # Test invalid field extraction
        getter = attrgetter('invalid_field')
        value = self.node.extract_value(test_msg, getter)
        self.assertIsNone(value)

        # Test processing without filter
        value = self.node.process_single_field("test_value", {})
        self.assertEqual(value, "test_value")

        # Test processing with passing filter
        mapping = {
            'mapping_options': {
                'filter': 'lambda x: len(x) > 3'
            }
        }
        value = self.node.process_single_field("test_value", mapping)
        self.assertEqual(value, "test_value")

        # Test processing with failing filter
        value = self.node.process_single_field("abc", mapping)
        self.assertIsNone(value)

    def test_process_array(self):
        # Test array of simple values
        simple_array = ["value1", "value2", "value3"]
        result = self.node.process_array(simple_array, {})
        self.assertEqual(json.loads(result), {"data": simple_array})

        # Test array with filter
        mapping_with_filter = {
            'mapping_options': {
                'filter': 'lambda x: len(x) > 6'
            }
        }
        result = self.node.process_array(simple_array, mapping_with_filter)
        self.assertEqual(json.loads(result), {"data": []})

        # Test array of objects with field extraction
        points = [
            Point(x=1.0, y=2.0, z=3.0),
            Point(x=4.0, y=5.0, z=6.0)
        ]
        mapping_with_fields = {
            'mapping_options': {
                'fields': ['x', 'y']
            }
        }
        result = self.node.process_array(points, mapping_with_fields)
        expected = {
            "data": [
                {"x": 1.0, "y": 2.0},
                {"x": 4.0, "y": 5.0}
            ]
        }
        self.assertEqual(json.loads(result), expected)

        # Test invalid field extraction
        mapping_with_invalid = {
            'mapping_options': {
                'fields': ['invalid_field']
            }
        }
        result = self.node.process_array(points, mapping_with_invalid)
        self.assertIsNone(result)

    def test_extract_values_as_dict(self):
        # Create a test message with nested fields
        class TestMsg:
            def __init__(self):
                self.nested = type('NestedMsg', (), {
                    'field1': 'value1',
                    'field2': 42,
                    'timestamp': Time(sec=1, nanosec=500000000)
                })()
        
        msg = TestMsg()
        
        # Test basic field extraction
        mapping = {
            'field': 'nested',
            'mapping_options': {
                'fields': ['field1', 'field2']
            }
        }
        result = self.node.extract_values_as_dict(msg, mapping)
        expected = {
            'field1': 'value1',
            'field2': 42
        }
        self.assertEqual(result, expected)
        
        # Test with time field
        mapping['mapping_options']['fields'] = ['timestamp']
        result = self.node.extract_values_as_dict(msg, mapping)
        print(f"\nTime value type: {type(result['timestamp'])}")
        print(f"Time value: {result['timestamp']}")
        self.assertEqual(result['timestamp'], 1.5)  # 1.5 seconds
        
        # Test with filter (passing)
        mapping['mapping_options'] = {
            'fields': ['field1', 'field2'],
            'filter': 'lambda x: x["field2"] > 40'
        }
        result = self.node.extract_values_as_dict(msg, mapping)
        self.assertEqual(result, expected)
        
        # Test with filter (failing)
        mapping['mapping_options']['filter'] = 'lambda x: x["field2"] < 40'
        result = self.node.extract_values_as_dict(msg, mapping)
        self.assertIsNone(result)
        
        # Test invalid field
        mapping['mapping_options'] = {
            'fields': ['invalid_field']
        }
        result = self.node.extract_values_as_dict(msg, mapping)
        self.assertEqual(result, {'invalid_field': None})

    def test_serialize(self):
        # Create a test message
        pose = Pose()
        pose.position = Point(x=1.0, y=2.0, z=3.0)
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        
        # Test full serialization
        mapping = {
            'mapping_options': {}
        }
        result = self.node.serialize(pose, mapping)
        self.assertIn('position', result)
        self.assertIn('orientation', result)
        self.assertEqual(result['position']['x'], 1.0)
        
        # Test with field filtering
        mapping = {
            'mapping_options': {
                'fields': ['position']
            }
        }
        result = self.node.serialize(pose, mapping)
        self.assertIn('position', result)
        self.assertNotIn('orientation', result)
        
        # Test with invalid message
        class InvalidMsg:
            pass
        
        result = self.node.serialize(InvalidMsg(), mapping)
        self.assertIsNone(result)

    def test_setup_republishers(self):
        """Test that publishers are created correctly"""
        test_config = """
        republishers:
          - topic: "/test_topic1"
            msg_type: "std_msgs/String"
            mappings:
              - field: "data"
                out:
                  topic: "/inorbit/custom_data"
                  key: "test_key1"
        """
        self.node.load_config(test_config)
        self.node.setup_republishers()
        
        # Simple verification that publishers exist
        self.assertIn("/inorbit/custom_data", self.node.pubs)

    def test_republisher_callback(self):
        """Test that messages are processed and published correctly"""
        # Create test message
        test_msg = String(data="test_value")
        
        # Simple mapping config
        repub = {
            'mappings': [
                {
                    'field': 'data',
                    'out': {
                        'topic': '/inorbit/custom_data',
                        'key': 'test_key1'
                    }
                }
            ]
        }
        
        # Create publisher
        self.node.create_topic_publisher('/inorbit/custom_data', 'std_msgs/String')
        
        # Create subscription to verify output
        received_messages = []
        def callback(msg):
            received_messages.append(msg.data)
        
        self.node.create_subscription(
            String,
            '/inorbit/custom_data',
            callback,
            10
        )
        
        # Process message
        self.node.republisher_callback(test_msg, repub, '/test_topic', False)
        
        # Spin a few times to process messages
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.1)
        
        # Verify output
        self.assertEqual(len(received_messages), 1)
        self.assertEqual(received_messages[0], 'test_key1=test_value')

    def test_setup_static_publishers(self):
        """Test static publisher configuration"""
        # Create a mock package structure
        with tempfile.TemporaryDirectory() as temp_dir:
            pkg_name = "test_package"
            pkg_version = "1.0.0"
            
            # Create ROS2 package structure
            os.makedirs(os.path.join(temp_dir, "share", pkg_name))
            
            # Create package.xml with version
            with open(os.path.join(temp_dir, "share", pkg_name, "package.xml"), 'w') as f:
                f.write(f"""<?xml version="1.0"?>
                <package format="3">
                  <name>{pkg_name}</name>
                  <version>{pkg_version}</version>
                  <description>Test package</description>
                  <license>MIT</license>
                  <maintainer email="test@test.com">Test User</maintainer>
                </package>
                """)
            
            # Add package to ament index
            os.makedirs(os.path.join(temp_dir, "share", "ament_index", "resource_index", "packages"))
            with open(os.path.join(temp_dir, "share", "ament_index", "resource_index", "packages", pkg_name), 'w') as f:
                f.write(os.path.join(temp_dir, "share", pkg_name))
            
            # Set ament prefix path
            original_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
            os.environ['AMENT_PREFIX_PATH'] = f"{temp_dir}:{original_prefix}"
            
            try:
                test_config = f"""
                static_publishers:
                  - out:
                      topic: "/inorbit/custom_data"
                      key: "version"
                    value_from:
                      package_version: "{pkg_name}"
                  - out:
                      topic: "/inorbit/custom_data"
                      key: "env_var"
                    value_from:
                      environment_variable: "TEST_ENV_VAR"
                  - out:
                      topic: "/inorbit/custom_data"
                      key: "static_value"
                    value: "fixed_value"
                """
                
                os.environ['TEST_ENV_VAR'] = 'test_value'
                self.node.load_config(test_config)
                
                # Track received messages
                received_messages = []
                def callback(msg):
                    received_messages.append(msg.data)
                
                self.node.create_subscription(
                    String,
                    '/inorbit/custom_data',
                    callback,
                    10
                )
                
                self.node.setup_static_publishers()
                
                # Spin with timeout
                timeout = time.time() + 5.0
                while len(received_messages) < 3 and time.time() < timeout:
                    self.executor.spin_once(timeout_sec=0.1)
                
                # Verify all messages
                self.assertIn(f'version={pkg_version}', received_messages)
                self.assertIn('env_var=test_value', received_messages)
                self.assertIn('static_value=fixed_value', received_messages)
                
            finally:
                # Restore original environment
                os.environ['AMENT_PREFIX_PATH'] = original_prefix

    def test_start(self):
        test_config = """
        republishers:
          - topic: "/test_topic"
            msg_type: "std_msgs/String"
            latched: true
            mappings:
              - field: "data"
                out:
                  topic: "/inorbit/custom_data"
                  key: "test_key"
        static_publishers:
          - out:
              topic: "/inorbit/custom_data"
              key: "static_value"
            value: "test_value"
        """
        
        self.node.load_config(test_config)
        self.node.start()
        
        # Give some time for publishers to be created
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.1)
        
        # Verify publishers were created
        self.assertIn("/inorbit/custom_data", self.node.pubs)
        self.assertIn("/inorbit/custom_data+/test_topic", self.node.pubs)

if __name__ == '__main__':
    unittest.main() 