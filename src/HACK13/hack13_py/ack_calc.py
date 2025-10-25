#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hack13.msg import Angle
from hack13.msg import AckState



class AckCalc(Node):
  def __init__(self):
    super().__init__("ack_calc")


def main(args=None):
  rclpy.init(args=None)

  node = AckCalc()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
