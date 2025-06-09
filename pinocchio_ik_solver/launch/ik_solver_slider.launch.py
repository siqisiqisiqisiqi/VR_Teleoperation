from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # Node 1: Set initial pose (runs immediately)
    initial_pose_node = Node(
        package='pinocchio_ik_solver',
        executable='initial_pose',
        name='initial_pose',
        output='screen'
    )

    # Other nodes (delayed 5 seconds)
    delayed_nodes = TimerAction(
        period=5.0,  # seconds
        actions=[
            Node(
                package='pinocchio_ik_solver',
                executable='slider_gui_pose',
                name='slider_gui_pose',
                output='screen'
            ),
            Node(
                package='pinocchio_ik_solver',
                executable='ik_solver',
                name='ik_solver',
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        initial_pose_node,
        delayed_nodes
    ])
