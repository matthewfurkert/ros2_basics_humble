#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from random import uniform
from functools import partial
from math import pi

from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import Turtle, AliveTurtles
from my_robot_interfaces.srv import CatchTurtle

class CreateTurtle():
    def __init__(self, name, x, y, theta):
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
  
class TurtleSpawnerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("turtle_spawner") # MODIFY NAME
        self.declare_parameter("spawn_frequency", 1.0)
        self.declare_parameter("turtle_name_prefix", "")
        self.alive_ = []
        self.dead_ = []
        self.turt_ = 2
        
        self.frequency_ = self.get_parameter("spawn_frequency").value
        self.name_prefix_ = self.get_parameter("turtle_name_prefix").value
        self.timer_ = self.create_timer(self.frequency_, self.set_values)
        self.publisher_ = self.create_publisher(AliveTurtles, "alive_turtles", 10)
        self.timer_ = self.create_timer(0.1, self.publish_turtles)
        self.server_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)
        
    def set_values(self):
        x = round(uniform(0, 11.05), 1)
        y = round(uniform(0, 11.05), 1)
        theta = round(uniform(-pi, pi), 2)
        self.call_turtle_spawner_server(x, y, theta)
    
    def call_turtle_spawner_server(self, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server \"spawn\"...")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = f"{self.name_prefix_}turtle{self.turt_}"
        self.turt_ +=1
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_turtle_spawner, x=x, y=y, theta=theta))
    
    def callback_turtle_spawner(self, future, x, y, theta):
        try:
            response = future.result()
            self.alive_.append(CreateTurtle(response.name, x, y, theta))
            self.get_logger().info(f"A turtle was spawned at x:{x}, y:{y} with the name {response.name}")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
 
    def publish_turtles(self):
        msg = AliveTurtles()
        msg.alive = []
        for turtle in self.alive_:
            turt_msg = Turtle()
            turt_msg.name = turtle.name
            turt_msg.x = turtle.x
            turt_msg.y = turtle.y
            turt_msg.theta = turtle.theta
            if turt_msg not in msg.alive:
                msg.alive.append(turt_msg)
        self.publisher_.publish(msg)
    
    def callback_catch_turtle(self, request, response):
        response.success = False
        for turtle in self.alive_:
            if turtle.name == request.name:
                self.alive_.remove(turtle)
                self.dead_.append(turtle)
                response.success = True
                self.call_kill_server(turtle)
        
        return response
    
    def call_kill_server(self, turtle):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server \"spawn\"...")
        
        request = Kill.Request()
        request.name = turtle.name
        client.call_async(request)
    
    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()