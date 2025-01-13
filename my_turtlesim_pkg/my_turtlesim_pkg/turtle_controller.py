#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_robot_interfaces.msg import AliveTurtles
from my_robot_interfaces.srv import CatchTurtle
from math import pi, sqrt
from functools import partial
 
class TurtleControllerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_controller") # MODIFY NAME
        self.x = 0
        self.y = 0
        self.theta = 0
        self.x_target = 5.5
        self.y_target = 5.5
        self.declare_parameter("catch_closest_turtle_first", False)
        
        self.catch_mode_ = self.get_parameter("catch_closest_turtle_first").value
        self.subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callback_boss_turtle_position, 10)
        self.subscriber_ = self.create_subscription(AliveTurtles, "alive_turtles", self.callback_turtles, 10)
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.1, self.publish_movement)
        self.get_logger().info("Turtle controller has started...")
    
    def callback_boss_turtle_position(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
    
    def callback_turtles(self, msg):
        min_dist = 99.0
        name = None
        if msg.alive == []:
            pass
        elif self.catch_mode_ == False:
            turtle = msg.alive[0]
            name = turtle.name
            min_dist = sqrt((self.x - turtle.x)**2 + (self.y - turtle.y)**2)
            self.x_target = turtle.x
            self.y_target = turtle.y
            self.get_logger().info(f"In ordered mode, the oldest turtle, {name} is {min_dist} units away")
        else:
            for turtle in msg.alive:
                dist = sqrt((self.x - turtle.x)**2 + (self.y - turtle.y)**2)
                if dist < min_dist:
                    min_dist = dist
                    name = turtle.name
                    self.x_target = turtle.x
                    self.y_target = turtle.y
                    self.get_logger().info(f"Closest turtle is {name} and is {min_dist} units away")
        if min_dist <= 0.5:
            self.call_catch_turtle_server(name)
    
    def publish_movement(self):
        dist = sqrt((self.x - self.x_target)**2 + (self.y - self.y_target)**2)
        msg = Twist()
        msg.linear.x = 5.0 if dist > 2.0 else 1.0
        if self.x > self.x_target:
            if self.y > self.y_target:
                if -0.75 * pi < self.theta < 0.25 * pi:
                    msg.angular.z = -2.0
                else:
                    msg.angular.z = 2.0
            else:
                if -0.25 * pi < self.theta < 0.75 * pi:
                    msg.angular.z = 2.0
                else:
                    msg.angular.z = -2.0
        else:
            if self.y > self.y_target:
                if -0.25 * pi < self.theta < 0.75 * pi:
                    msg.angular.z = -2.0
                else:
                    msg.angular.z = 2.0
            else:
                if -0.75 * pi < self.theta < 0.25 * pi:
                    msg.angular.z = 2.0
                else:
                    msg.angular.z = -2.0
        self.publisher_.publish(msg)
    
    def call_catch_turtle_server(self, name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server \"catch_turtle\"...")
        
        request = CatchTurtle.Request()
        request.name = name
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_catch_turtle, name=name))
    
    def callback_catch_turtle(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(f"{name} was removed ({response.success})")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 

if __name__ == "__main__":
    main()