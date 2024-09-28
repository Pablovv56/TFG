#!/usr/bin/env python3

import rclpy
import location_module as lm

def main (args = None):

    rclpy.init(args = args)

    node = lm.LocationNode("/noisy_cloud")
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()