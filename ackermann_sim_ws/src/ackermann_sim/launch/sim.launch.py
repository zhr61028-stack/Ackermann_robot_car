from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ackermann_sim.assets import generate


def setup(context):
    share=Path(get_package_share_directory('ackermann_sim'))
    config=LaunchConfiguration('config').perform(context)
    generated=Path(tempfile.mkdtemp(prefix='ackermann_sim_'))
    generate(share,config,generated)
    gui=LaunchConfiguration('gui')
    common={'use_sim_time':True}
    actions=[ExecuteProcess(cmd=['gz','sim','-r','-s',str(generated/'test_world.sdf')],output='screen'),
        ExecuteProcess(cmd=['gz','sim','-g','--gui-config',str(share/'config/gui.config')],condition=IfCondition(gui),output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='sim_bridge',parameters=[{'config_file':str(generated/'bridge.yaml')}],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[common,{'robot_description':(generated/'vehicle.urdf').read_text()}],output='screen'),
        Node(package='ackermann_sim',executable='command_guard',parameters=[common,{'config_path':config}],output='screen'),
        Node(package='ackermann_sim',executable='path_observer',parameters=[common],output='screen'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/ackermann.rviz')],parameters=[common],condition=IfCondition(LaunchConfiguration('rviz')),output='screen')]
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory('ackermann_sim'))
    return LaunchDescription([DeclareLaunchArgument('config',default_value=str(share/'config/vehicle.yaml')),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
