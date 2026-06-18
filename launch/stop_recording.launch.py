from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the configurable argument for the component ID
    component_id_arg = DeclareLaunchArgument(
        'component_id',
        default_value='7',
        description='The ID of the component to unload'
    )

    # Action 1: Call the ZED service
    # We use 'ros2 service call' directly
    stop_svo_rec = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/zed/zed_node/stop_svo_rec', 'std_srvs/srv/Trigger'],
        output='screen'
    )

    # Action 2: Unload the component
    # We use the LaunchConfiguration to substitute the 'component_id' argument
    unload_component = ExecuteProcess(
        cmd=[
            'ros2', 'component', 'unload', 
            '/saberguide', 
            LaunchConfiguration('component_id')
        ],
        output='screen'
    )

    return LaunchDescription([
        component_id_arg,
        stop_svo_rec,
        unload_component
    ])