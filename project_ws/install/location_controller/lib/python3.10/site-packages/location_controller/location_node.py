#!/usr/bin/env python3

import rclpy
from location_controller.location_module import LocationNode
from location_controller.noisy_cloud import NoisyCloud

def main (args = None):

    rclpy.init(args = args)

    node = LocationNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()