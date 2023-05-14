import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

class bcolors:
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

def generate_launch_description():
    ld = LaunchDescription()
    spiceypy_cassini_config = os.path.join(
        get_package_share_directory('spiceypy_cassini'),
        'config',
        'params.yaml'
        )
    spiceypy_cassini=Node(
        package = 'spiceypy_cassini',
        name = 'spiceypy_cassini',
        executable = 'spiceypy_cassini',
        parameters = [spiceypy_cassini_config]
    )

    sys_shut_down = RegisterEventHandler(OnProcessExit(
    target_action=spiceypy_cassini,
    on_exit=[
                LogInfo(msg=(f'{bcolors.OKGREEN}The Scenario has ended!{bcolors.ENDC}')),
                EmitEvent(event=Shutdown(
                    reason='Finished'))
            ]		
    ))

    bridge_dir = get_package_share_directory('rosbridge_server')
    bridge_launch =  IncludeLaunchDescription(launch_description_sources.FrontendLaunchDescriptionSource(bridge_dir + '/launch/rosbridge_websocket_launch.xml')) 
    ld = LaunchDescription ([bridge_launch, spiceypy_cassini, sys_shut_down])
    return ld