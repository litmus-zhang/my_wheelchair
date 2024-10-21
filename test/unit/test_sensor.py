#!/usr/bin/env python3
import pytest
from my_wheelchair.sensors_node import SensorsNode
import rclpy
from rclpy.node import Node

class TestMyNode:
    @classmethod
    def setup_class(cls):
        # Initialize ROS 2 context for tests
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        # Shutdown ROS 2 context after tests
        rclpy.shutdown()

    def setup_method(self):
        # Create a new node for each test
        self.node = SensorsNode()

    def teardown_method(self):
        # Cleanup after each test
        self.node.destroy_node()

    def test_node_name(self):
        """Test if the node has the correct name"""
        assert self.node.get_name() == 'sensors_node'
