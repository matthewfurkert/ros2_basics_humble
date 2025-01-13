#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import BatteryStatus
from functools import partial
 
 
class BatteryNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("battery") # MODIFY NAME
        self.battery_state_ = "full"
        self.battery_state_changed_time_ = self.get_current_time()
        self.timer_ = self.create_timer(0.1, self.check_battery_state)
    
    def get_current_time(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0
    
    def check_battery_state(self):
        if self.battery_state_ == "full":
            if self.get_current_time() - self.battery_state_changed_time_ > 4.0:
                self.battery_state_changed_time_ = self.get_current_time()
                self.battery_state_ = "empty"
                self.call_set_led_server(self.battery_state_)
        else:
            if self.get_current_time() - self.battery_state_changed_time_ > 6.0:
                self.battery_state_changed_time_ = self.get_current_time()
                self.battery_state_ = "full"
                self.call_set_led_server(self.battery_state_)
    
    def call_set_led_server(self, status):
        client = self.create_client(BatteryStatus, "battery_status")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server add Two Ints...")
        
        request = BatteryStatus.Request()
        request.state = status
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_led_status, a=status))
        
    def callback_led_status(self, future, a):
        try:
            response = future.result()
            self.get_logger().info(f"{a} battey status was sent to the server and a {response.success} was recieved")
        except Exception as e:
            self.get_logger().error("Service called failed %r" % (e,))
 
 
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()