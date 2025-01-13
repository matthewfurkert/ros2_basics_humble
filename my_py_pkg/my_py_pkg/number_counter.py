#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("number_counter") # MODIFY NAME
        self.count = 1
        self.total = 0
        self.message = "No message received yet"
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_number_counter, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_set_bool)
        self.get_logger().info("Number counter has been started")
        self.publish_num()
    
    def callback_number_counter(self, msg):
        self.message = f"Received \"{msg.data}\" {self.count} times"
        self.get_logger().info(self.message)
        self.count += 1
        self.total += msg.data
        self.publish_num()
    
    def publish_num(self):
        msg = Int64()
        msg.data = self.total
        self.publisher_.publish(msg)
    
    def callback_set_bool(self, request, response):
        if request.data:
            response.success = True
            response.message = "Counter reset"
            self.total = 0
        else:
            response.success = False
            response.message = "Counter not reset"
        return response
        
 
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()