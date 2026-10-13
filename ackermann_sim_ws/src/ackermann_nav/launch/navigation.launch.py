from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ackermann_nav.assets import generate

def setup(context):
 sim=Path(get_package_share_directory('ackermann_sim'));nav=Path(get_package_share_directory('ackermann_nav'))
 vehicle=LaunchConfiguration('vehicle').perform(context);scene=LaunchConfiguration('scene').perform(context)
 out=Path(tempfile.mkdtemp(prefix='ackermann_nav_'));generate(sim,nav,vehicle,scene,out)
 common={'use_sim_time':True};params=str(out/'nav2.yaml')
 nodes=[ExecuteProcess(cmd=['gz','sim','-r','-s',str(out/'test_world.sdf')],output='screen'),
  ExecuteProcess(cmd=['gz','sim','-g','--gui-config',str(nav/'config/gui.config')],condition=IfCondition(LaunchConfiguration('gui')),output='screen'),
  Node(package='ros_gz_bridge',executable='parameter_bridge',name='sim_bridge',parameters=[{'config_file':str(out/'bridge.yaml')}],output='screen'),
  Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[common,{'robot_description':(out/'vehicle.urdf').read_text()}],output='screen'),
  Node(package='ackermann_sim',executable='command_guard',parameters=[common,{'config_path':vehicle,'allow_reverse':False}]),
  Node(package='ackermann_sim',executable='path_observer',parameters=[common],remappings=[('/odom','/wheel/odom')]),
  Node(package='ackermann_nav',executable='truth_localization',parameters=[common]),
  Node(package='rviz2',executable='rviz2',arguments=['-d',str(nav/'rviz/ackermann.rviz')],parameters=[common],condition=IfCondition(LaunchConfiguration('rviz')),output='screen')]
 servers=[('nav2_map_server','map_server'),('nav2_planner','planner_server'),('nav2_controller','controller_server'),('nav2_behaviors','behavior_server'),('nav2_bt_navigator','bt_navigator')]
 for pkg,name in servers:nodes.append(Node(package=pkg,executable=name,name=name,parameters=[params],output='screen'))
 nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',parameters=[common,{'autostart':True,'node_names':[n for _,n in servers],'bond_timeout':4.0}],output='screen'))
 return nodes

def generate_launch_description():
 sim=Path(get_package_share_directory('ackermann_sim'));nav=Path(get_package_share_directory('ackermann_nav'))
 return LaunchDescription([DeclareLaunchArgument('vehicle',default_value=str(sim/'config/vehicle.yaml')),DeclareLaunchArgument('scene',default_value=str(nav/'config/test_scene.yaml')),DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),OpaqueFunction(function=setup)])
