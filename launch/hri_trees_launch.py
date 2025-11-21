import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg

def generate_launch_description():
    # Get the path to your behavior tree XML file
    bt_xml_file_path = os.path.join(
        get_package_share_directory('hri_trees'),
        'trees',
        'hospibot.xml'
    )

    # Create the lifecycle node
    lifecycle_node = LifecycleNode(
        package='my_bt_package',
        executable='bt_executor',
        name='bt_executor_node',
        namespace='',
        output='screen',
        parameters=[{'bt_xml_file': bt_xml_file_path}]
    )

    # Event handler to configure the node after it starts
    register_event_handler_for_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                ChangeState(
                    lifecycle_node_matcher=lambda node: node == lifecycle_node,
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                ),
            ],
        )
    )

    # Event handler to activate the node after it is configured
    register_event_handler_for_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                ChangeState(
                    lifecycle_node_matcher=lambda node: node == lifecycle_node,
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                ),
            ],
        )
    )
    
    return LaunchDescription([
        lifecycle_node,
        register_event_handler_for_configure,
        register_event_handler_for_activate,
    ])