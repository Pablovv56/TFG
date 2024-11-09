#!/usr/bin/env python3

import rclpy
import location_controller.location_module as lcm

def main (args = None):

    rclpy.init(args = args)

    node = lcm.LocationNode("/cloud_in")
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()