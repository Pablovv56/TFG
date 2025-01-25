#!/usr/bin/env python3

import rclpy
from location_controller.location_module import LocationNode
def main (args = None):

    rclpy.init(args = args)

    node = LocationNode("/cloud_in")
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()