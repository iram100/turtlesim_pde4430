from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # start turtlesim (separate process so GUI opens)
    turtlesim_node = ExecuteProcess(
        cmd=['ros2', 'run', 'turtlesim', 'turtlesim_node'],
        output='screen'
    )

    # spawn multiple turtles after a short delay
    spawn_multi = TimerAction(
        period=1.0,
        actions=[Node(package='turtlesim_pde4430', executable='spawn_multi', name='spawn_multi')]
    )

    # start a straight_line publisher after turtlesim starts
    straight = TimerAction(
        period=0.5,
        actions=[Node(package='turtlesim_pde4430', executable='straight_line', name='straight_line')]
    )

    return LaunchDescription([
        turtlesim_node,
        spawn_multi,
        straight,
    ])
