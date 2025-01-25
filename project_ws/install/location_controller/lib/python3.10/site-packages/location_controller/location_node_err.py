#!/usr/bin/env python3

import rclpy
import location_controller.location_module

def main (args = None):

    rclpy.init(args = args)

    node = location_module.LocationNode("/noisy_cloud")
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()