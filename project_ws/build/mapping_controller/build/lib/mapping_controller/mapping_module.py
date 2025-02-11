#!/usr/bin/env python3

import rclpy
import Bonxai

from rclpy.node import Node

class MappingNode (Node):
    
    def __init__(self, input_topic : str):
        
        super.__init__("mapping_node")
        