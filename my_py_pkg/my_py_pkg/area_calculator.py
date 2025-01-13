#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import ComputeRectangleArea

class RectangleAreaServerNode(Node): # Modify Name
    def __init__(self):
        super().__init__("rectangle_area_server") # Modify Name
        self.server_ = self.create_service(ComputeRectangleArea, "rectangle_area", self.callback_rectangle_area)
        self.get_logger().info("Rectangle area server has been started")
    
    def callback_rectangle_area(self, request, response):
        response.area = request.length * request.width
        self.get_logger().info(f"{request.length} x {request.width} = {response.area}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RectangleAreaServerNode() # Modify Name
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()