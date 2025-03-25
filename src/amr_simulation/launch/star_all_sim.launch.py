from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('amr_simulation'),  
            '/launch/gazebo_sim.launch.py'
        ])
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('amr_simulation'),  
            '/launch/rviz_sim.launch.py'
        ])
    )

    return LaunchDescription([
        gazebo_launch,
        rviz_launch
    ])
