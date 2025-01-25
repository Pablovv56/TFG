#!/usr/bin/env python3

import rclpy
from location_controller.noisy_cloud_module import NoisyCloud

def main (args = None):

    rclpy.init(args = args)

    node = NoisyCloud("/cloud_in")
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
    