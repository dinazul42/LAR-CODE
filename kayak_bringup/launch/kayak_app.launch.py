from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # autopilot_pkg nodes
    guidance_node = Node(
        package='autopilot_pkg',
        executable='guidance_node'
    )

    parameters_node = Node(
        package='autopilot_pkg',
        executable='parameters_node'
    )

    live_node = Node(
        package='autopilot_pkg',
        executable='live_node'
    )

    mission_control_node = Node(
        package='autopilot_pkg',
        executable='mission_control_node'
    )

    qgc_comm_node = Node(
        package='autopilot_pkg',
        executable='qgc_comm_node'
    )

    # i2c_devices_pkg nodes
    bme280_node = Node(
        package='i2c_devices_pkg',
        executable='bme280_node'
    )

    i2c_devices_publisher_node = Node(
        package='i2c_devices_pkg',
        executable='i2c_devices_publisher_node'
    )

    # motors_pkg nodes
    motors_node = Node(
        package='motors_pkg',
        executable='motors_node'
    )
    actuator_node = Node(
        package='motors_pkg',
        executable='actuator_control_node'
    )
    # gps_pkg nodes
    gps_node = Node(
        package='gps_pkg',
        executable='gps_node'
    )

    # Add nodes to LaunchDescription
    ld.add_action(guidance_node)
    ld.add_action(parameters_node)
    ld.add_action(live_node)
    ld.add_action(mission_control_node)
    ld.add_action(qgc_comm_node)
    ld.add_action(bme280_node)
    ld.add_action(i2c_devices_publisher_node)
    ld.add_action(motors_node)
    ld.add_action(gps_node)

    return ld
