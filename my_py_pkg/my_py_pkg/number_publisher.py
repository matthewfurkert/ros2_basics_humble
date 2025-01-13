#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
 
class NumberPublisherNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("number_publisher") # MODIFY NAME
        self.declare_parameter("number_to_publish", 2)
        self.declare_parameter("publish_frequency", 1.0)
        
        self.number_ = self.get_parameter("number_to_publish").value
        self.frequency_ = self.get_parameter("publish_frequency").value
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(1.0 / self.frequency_, self.publish_num)
        self.get_logger().info("Robot number publisher has been started")
    
    def publish_num(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()