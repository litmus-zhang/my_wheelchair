#!/usr/bin/env python3
import pytest
from my_wheelchair.navigation_node import NavigationNode
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
        self.node = NavigationNode()

    def teardown_method(self):
        # Cleanup after each test
        self.node.destroy_node()

    def test_node_name(self):
        """Test if the node has the correct name"""
        assert self.node.get_name() == 'navigation_node'
