#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import Battery
from my_robot_interfaces.srv import BatteryStatus 
 
class LedPanelNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("led_panel") # MODIFY NAME
        self.declare_parameter("led_states", ["off", "off", "off"])
        self.leds = self.get_parameter("led_states").value
        
        self.server_ = self.create_service(BatteryStatus, "battery_status", self.callback_battery_status)
        self.publisher_ = self.create_publisher(Battery, "led_panel_state", 10)
        self.get_logger().info("Battery status server node has been started")
    
    def callback_battery_status(self, request, response):
        if request.state == "empty":
            self.leds[2] = "on"
        else:
            self.leds[2] = "off"
        if request.state not in ["full", "empty"]:
            response.success = False
        else:
            response.success = True
        self.get_logger().info(f"The battery signalled that it is {request.state}, \
            so led 3 is {self.leds[2]} and the {response.success} response was sent")
        
        self.publish_state()
        return response
    
    def publish_state(self):
        msg = Battery()
        msg.led_num = [0, 1, 2]
        msg.state = list(self.leds)
        self.publisher_.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()