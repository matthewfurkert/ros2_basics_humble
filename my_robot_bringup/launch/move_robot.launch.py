from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    ld = LaunchDescription()
    
    controller_node1 = LifecycleNode(
        package = "actions_py",
        executable = "move_robot_lifecycle_server",
        name = "move_robot_server1",
        namespace = "",
        parameters = [{"action_name" : "controller_1"}]
    )
    controller_node2 = LifecycleNode(
        package = "actions_py",
        executable = "move_robot_lifecycle_server",
        name = "move_robot_server2",
        namespace = "",
        parameters = [{"action_name" : "controller_2"}]
    )
    
    manager_node1 = Node(
        package="actions_py",
        executable = "lifecycle_node_manager",
        name = "manager_1",
        parameters=[{"managed_node_name" : "move_robot_server1"}]
    )
    manager_node2 = Node(
        package="actions_py",
        executable = "lifecycle_node_manager",
        name = "manager_2",
        parameters=[{"managed_node_name" : "move_robot_server2"}]
    )
    
    ld.add_action(controller_node1)
    ld.add_action(controller_node2)
    ld.add_action(manager_node1)
    ld.add_action(manager_node2)
    
    return ld