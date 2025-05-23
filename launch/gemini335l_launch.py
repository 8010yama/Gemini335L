from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # 引数の宣言（最小限）
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'),
    ]

    # パラメータ設定（最小構成で動作確認済みのもの）
    parameters = [{
        'camera_name': LaunchConfiguration('camera_name'),
        'enable_color': True,
        'enable_depth': True,
        'depth_width': 640,
        'depth_height': 480,
        'depth_fps': 30,
        'depth_format': 'Y16',
        'color_width': 640,
        'color_height': 480,
        'color_fps': 30,
        'color_format': 'MJPG',
    }]

    # カメラ用コンポーザブルノード
    camera_node = ComposableNode(
        package='orbbec_camera',
        plugin='orbbec_camera::OBCameraNodeDriver',
        name=LaunchConfiguration('camera_name'),
        parameters=parameters,
    )

    # コンテナの定義
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[camera_node],
        output='screen',
    )

    # PointCloud変換ノード（2秒遅延で起動）
    depth_to_pc_node = Node(
        package='gemini335l',
        executable='depth_to_pointcloud_node',
        name='depth_to_pointcloud_node',
        output='screen',
    )

    return LaunchDescription(
        args + [
            container,
            TimerAction(period=2.0, actions=[depth_to_pc_node]),  # ← 遅延起動で安全に
        ]
    )
