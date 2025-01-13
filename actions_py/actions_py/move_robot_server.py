#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class MoveRobotServerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("move_robot_server") # MODIFY NAME
        self.robot_position_ = 50
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.move_robot_server_ = ActionServer(
            self,
            MoveRobot,
            "move_robot",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback)
        self.get_logger().info("Action server has been started")
        self.get_logger().info(f"Robot position: {self.robot_position_}")
    
    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Received a goal")
        
        # Validate the goal request
        if not (0 <= goal_request.position <= 100 and 0 <= goal_request.velocity <= 100):
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT
        
        #Policy prempt existing goal when receiving new goal
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new goal")
                self.goal_handle_.abort()
        
        self.get_logger().info("Accepting a goal")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle    
        
        # Get request from goal
        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity
        
        # Execute the action
        self.get_logger().info("Executing the goal")
        feedback = MoveRobot.Feedback()
        result = MoveRobot.Result()
        while goal_position != self.robot_position_:
            if not goal_handle.is_active:
                result.position = self.robot_position_
                return result
            
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal")
                goal_handle.canceled()
                result.position = self.robot_position_
                result.message = "Robot movement cancelled"
                return result
                
            if goal_position > self.robot_position_:
                if (goal_position - self.robot_position_) > velocity:
                    self.robot_position_ += velocity
                    self.get_logger().info(f"{self.robot_position_}")
                else:
                    self.robot_position_ = goal_position
                    self.get_logger().info(f"{self.robot_position_}")
            elif goal_position < self.robot_position_:
                if (self.robot_position_ - goal_position) > velocity:
                    self.robot_position_ -= velocity
                    self.get_logger().info(f"{self.robot_position_}")
                else:
                    self.robot_position_ = goal_position
                    self.get_logger().info(f"{self.robot_position_}")
            feedback.current_position = self.robot_position_
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)
        
        # Once done, set the goal final state
        goal_handle.succeed()
        
        # and send the result
        result.position = self.robot_position_
        result.message = "Robot has arrived at destination"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode() # MODIFY NAME
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
