#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    # 1) Start turtlesim
    ld.add_action(Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    ))

    # 2) Spawn all turtles via Python spawner
    ld.add_action(Node(
        package='turtle_formation',
        executable='spawn_turtles',
        name='spawner',
        output='screen'
    ))

    # 3) After a delay, launch prey and predator nodes
    delay = 1.0
    # Prey
    ld.add_action(TimerAction(
        period=delay,
        actions=[Node(
            package='turtle_formation',
            executable='prey_node',
            name='prey_node',
            output='screen'
        )]
    ))
    # Predators
    num_agents = 5
    for i in range(1, num_agents+1):
        ld.add_action(TimerAction(
            period=delay,
            actions=[Node(
                package='turtle_formation',
                executable='predator_swarm',
                name=f'predator_{i}',
                parameters=[{'index': i, 'num_agents': num_agents}],
                output='screen'
            )]
        ))

    return ld
