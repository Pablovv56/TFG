#!/usr/bin/env python3

import rclpy
from mapping_controller.mapping_module import MappingNode
def main (args = None):

    rclpy.init(args = args)

    # Creates the node using the orginal or noisiy pcd as input
    node = MappingNode("/cloud_in")
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()