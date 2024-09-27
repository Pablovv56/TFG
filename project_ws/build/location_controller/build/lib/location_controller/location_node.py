#!/usr/bin/env python3

import rclpy
import location_controller.location_module as loc_module

def main (args = None):

    rclpy.init(args = args)

    node = loc_module.LocationNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()