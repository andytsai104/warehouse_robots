import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- 1. 定義套件名稱和 URDF 檔案名稱 ---
    # 請根據您實際建立的套件名稱和 URDF 檔案名稱修改這兩行
    package_name = 'warehouse_robots' # package 名稱
    xacro_file_name = 'warehouse_robots.urdf.xacro' # URDF 檔案名稱
    
    # 建立 URDF 檔案的完整路徑
    xacro_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        xacro_file_name
    )

    # 從 URDF 檔案讀取機器人描述內容
    robot_description = Command(['xacro ', xacro_path])
    
    # --- 2. 定義啟動參數 ---
    # 允許從命令行傳入模型路徑，但提供預設值
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=xacro_path,
        description='Path to robot xacro file'
    )

    # --- 3. 啟動節點 ---

    # 節點 A: robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # --- 【重要修正】直接使用定義好的變數，並確保使用 ParameterValue ---
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}]
    )

    # 節點 B: joint_state_publisher_gui (提供 GUI 滑桿來控制關節)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    
    # 節點 C: RViz (視覺化工具)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        
        # --- 可選：指定 RViz 配置檔案路徑 ---
        # arguments=['-d', os.path.join(
        #     get_package_share_directory(package_name),
        #     'rviz',
        #     'display.rviz'
        # )]
    )

    # --- 4. 返回 LaunchDescription ---
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])