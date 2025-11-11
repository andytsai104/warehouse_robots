import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'warehouse_robots'
    urdf_file_name = 'warehouse_robots.urdf'
    world_name = 'warehouse_world.sdf'
    
    warehouse_share = get_package_share_directory(package_name)

    urdf_path = os.path.join(warehouse_share, 'urdf', urdf_file_name)
    world_path = os.path.join(warehouse_share, 'world', world_name)

    # !!!share 上層目錄!!!
    warehouse_root = os.path.dirname(warehouse_share)  # Gazebo 將從這裡找到 warehouse_robots/

    # 新增 Ignition 資源搜尋路徑
    set_res_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
            os.pathsep,
            warehouse_root
        ]
    )

    # 從 URDF 讀取描述
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # ros_gz_sim 目錄
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # 啟動 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # spawn robot
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='gz_spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'warehouse_robot',
            '-x', '0.0', '-y', '-2.0', '-z', '0.1'
        ]
    )

    return LaunchDescription([
        set_res_path,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node
    ])
