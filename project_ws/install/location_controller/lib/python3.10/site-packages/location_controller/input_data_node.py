#!/usr/bin/env python3

import rclpy
from location_controller.input_data_module import InputData
def main (args = None):

    rclpy.init(args = args)

    # Creates the node using the orginal or noisiy pcd as input
    node = InputData()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()