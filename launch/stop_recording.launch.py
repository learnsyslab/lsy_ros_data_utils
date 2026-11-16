from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# Component ids are handed out in load order and never reused, so the recorder's
# id changes with every start_recording (and with the order in which the sensor
# components finish loading). Look it up by node name instead of hardcoding it.
UNLOAD_RECORDER = r'''
container="$0"; node="$1"; id="$2"
if [ -z "$id" ]; then
  id=$(ros2 component list "$container" | awk -v n="$node" '$2 == n {print $1}')
fi
if [ -z "$id" ]; then
  echo "stop_recording: no $node loaded in $container - nothing to unload"
  exit 1
fi
rc=0
for i in $id; do
  ros2 component unload "$container" "$i" || rc=1
done
exit $rc
'''


def generate_launch_description():
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='saberguide',
        description='The container the recorder component was loaded into'
    )

    component_name_arg = DeclareLaunchArgument(
        'component_name',
        default_value='bag_recorder_component',
        description='Node name of the recorder component to unload'
    )

    # Optional override; empty means: find the id by component_name
    component_id_arg = DeclareLaunchArgument(
        'component_id',
        default_value='',
        description='The ID of the component to unload (default: looked up by name)'
    )

    # Action 1: Call the ZED service
    # We use 'ros2 service call' directly
    stop_svo_rec = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/zed/zed_node/stop_svo_rec', 'std_srvs/srv/Trigger'],
        output='screen'
    )

    # Action 2: Unload the recorder component (all instances with that name)
    unload_component = ExecuteProcess(
        cmd=[
            'bash', '-c', UNLOAD_RECORDER,
            ['/', LaunchConfiguration('container_name')],
            ['/', LaunchConfiguration('component_name')],
            LaunchConfiguration('component_id'),
        ],
        output='screen'
    )

    return LaunchDescription([
        container_name_arg,
        component_name_arg,
        component_id_arg,
        stop_svo_rec,
        unload_component
    ])
