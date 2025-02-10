import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):
    # rtabmap 관련 파라미터 (모두 문자열 literal로 지정)
    rtabmap_parameters = {
        "Grid/3D": "true",
        "Grid/Sensor": "2",
        "Grid/RayTracing": "false",
        "Grid/CellSize": "0.05",
        "Reg/Strategy": "2",
        "ICP/Strategy": "1",
        "Optimizer/Strategy": "2",
        "Vis/FeatureType": "8",
        "Vis/EstimationType": "0",
        "RGBD/CreateOccupancyGrid": "false",
        "RGBD/LoopClosureReextractFeatures": "True",
        "RGBD/ProximityPathMaxNeighbors": "5",
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "true",
        "Mem/DepthCompressionFormat": ".png",
        "Vis/MinInliers": "20",
    }
    
    # Gazebo 시뮬레이터 실행 (모바일 매니퓰레이터 패키지 내 gazebo.launch.py 사용)
    gazebo_sim_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        get_package_share_directory('mobile_manipulator'),
                        'launch',
                        'gazebo.launch.py'
                    ])
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "launch_rviz": "false",
                    "gazebo_gui": "false",
                    "select_lidar": "3d",
                    "select_rear_wheel_bracket": "true",
                }.items()
            )
        ]
    )
    
    # base camera용 rgbd_sync 노드
    bc_rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        namespace='/base_camera',
        output="screen",
        emulate_tty=True,
        parameters=[{
            "approx_sync": True,
            "approx_sync_max_interval": 0.5,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "qos": 0,
            "qos_camera_info": 0,
            "depth_scale": 1.0,
        }],
        remappings=[
            ("rgb/image", "/base_camera_color_sensor/image_raw"),
            ("rgb/camera_info", "/base_camera_color_sensor/camera_info"),
            ("depth/image", "/base_camera_depth_sensor/depth/image_raw"),
            ("depth/camera_info", "/base_camera_depth_sensor/depth/camera_info"),
            ("rgbd_image", "rgbd_image"),
        ],
    )
    
    # gripper camera용 rgbd_sync 노드
    gc_rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        namespace='/gripper_camera',
        output="screen",
        emulate_tty=True,
        parameters=[{
            "approx_sync": True,
            "approx_sync_max_interval": 0.5,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "qos": 0,
            "qos_camera_info": 0,
            "depth_scale": 1.0,
        }],
        remappings=[
            ("rgb/image", "/gripper_camera_color_sensor/image_raw"),
            ("rgb/camera_info", "/gripper_camera_color_sensor/camera_info"),
            ("depth/image", "/gripper_camera_depth_sensor/depth/image_raw"),
            ("depth/camera_info", "/gripper_camera_depth_sensor/depth/camera_info"),
            ("rgbd_image", "rgbd_image"),
        ],
    )
    
    # Base camera RGB-D odometry 노드 (publish_tf 비활성화)
    bc_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        namespace='/base_camera',
        output="screen",
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "publish_tf": False,
            "approx_sync": True,
            "approx_sync_max_interval": 0.5,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "subscribe_rgbd": True,
            "subscribe_rgb": False,
            "subscribe_depth": False,
            "wait_for_transform": 0.2,
            "wait_imu_to_init": False,
            "always_check_imu_tf": True,
        }],
        remappings=[
            ("rgbd_image", "rgbd_image"),
            ("odom", "odom"),
            ("imu", "/base_camera_imu_sensor/imu/data"),
        ],
        arguments=["--ros-args", "--log-level", "/base_camera.rgbd_odometry:=info", "--log-level", "rgbd_odometry:=info"],
        prefix='',
    )
    
    # Gripper camera RGB-D odometry 노드 (publish_tf 비활성화)
    gc_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        namespace='/gripper_camera',
        output="screen",
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "publish_tf": False,
            "approx_sync": True,
            "approx_sync_max_interval": 0.5,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "subscribe_rgbd": True,
            "subscribe_rgb": False,
            "subscribe_depth": False,
            "wait_for_transform": 0.2,
            "wait_imu_to_init": False,
            "always_check_imu_tf": True,
        }],
        remappings=[
            ("rgbd_image", "rgbd_image"),
            ("odom", "odom"),
        ],
        arguments=["--ros-args", "--log-level", "/gripper_camera.rgbd_odometry:=info", "--log-level", "rgbd_odometry:=info"],
        prefix='',
    )
    
    # ICP odometry 노드 (publish_tf 비활성화)
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        namespace="/icp_odom",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "publish_tf": False,
            "wait_for_transform": 0.2,
            "wait_imu_to_init": False,
            "always_check_imu_tf": True,
            "approx_sync": True,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "qos": 0,
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
        }],
        remappings=[
            ("scan", "/dummy"),
            ("scan_cloud", "/lidar/points"),
            ("odom", "odom"),
        ],
        arguments=["--ros-args", "--log-level", "/icp_odom.icp_odometry:=info", "--log-level", "icp_odometry:=info"],
        prefix='',
    )
    
    # UKF 노드 (UKF 설정 파일 경로를 literal로 사용)
    ukf_config_path = os.path.join(get_package_share_directory('rtabmap_launch'),
                                   'launch', 'config', 'ukf.yaml')
    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        output='screen',
        parameters=[ukf_config_path],
        remappings=[
            ('odometry/filtered', '/ukf_odom/odom'),
            ('accel/filtered', '/ukf_odom/accel'),
        ],
    )
    
    # rtabmap SLAM 노드 (SLAM이 publish_tf하여 map frame 기반 좌표계 생성)
    slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        namespace="/rtabmap",
        output="screen",
        parameters=[rtabmap_parameters, {
            "rgbd_cameras": 2,
            "subscribe_depth": False,
            "subscribe_rgbd": True,
            "subscribe_rgb": False,
            "subscribe_stereo": False,
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
            "subscribe_odom_info": True,
            "frame_id": "base_link",
            "map_frame_id": "map",
            "odom_frame_id": "odom",
            "publish_tf": True,
            "initial_pose": "0.0, 0.0, 0.0, 0.0, 0.0, -0.735398",
            "odom_tf_angular_variance": 0.01,
            "odom_tf_linear_variance": 0.001,
            "odom_sensor_sync": True,
            "wait_for_transform": 0.2,
            "database_path": "~/hc_ws/src/RTAB_Map/rtabmap_db/test.db",
            "approx_sync": True,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "qos_image": 0,
            "qos_scan": 0,
            "qos_odom": 0,
            "qos_camera_info": 0,
            "qos_imu": 0,
            # "qos_gps": 0,
            "scan_normal_k": 20,
            "landmark_linear_variance": 0.0001,
            "landmark_angular_variance": 9999.0,
            
        }],
        remappings=[
            ("rgbd_image0", "/base_camera/rgbd_image"),
            ("rgbd_image1", "/gripper_camera/rgbd_image"),
            ("scan_cloud", "/lidar/points"),
            ("odom", "ukf_odom/odom"),
        ],
        # arguments=["--ros-args", "--log-level", "/rtabmap.rtabmap:=info", "--log-level", "rtabmap:=info"],
        prefix='',
    )
    
    slam_node_delayed = TimerAction(
        period=1.0,
        actions=[slam_node]
    )
    
    # rtabmap visualization 노드
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        namespace="/rtabmap",
        output='screen',
        emulate_tty=True,
        parameters=[{
            "rgbd_cameras": 2,
            "subscribe_depth": False,
            "subscribe_rgbd": True,
            "subscribe_rgb": False,
            "subscribe_stereo": False,
            "subscribe_scan": False,
            "subscribe_scan_cloud": True,
            "subscribe_odom_info": True,
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "wait_for_transform": 0.2,
            "approx_sync": True,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "qos_image": 0,
            "qos_scan": 0,
            "qos_odom": 0,
            "qos_camera_info": 0,
        }],
        remappings=[
            ("rgbd_image0", "/base_camera/rgbd_image"),
            ("rgbd_image1", "/gripper_camera/rgbd_image"),
            ("scan", "/dummy"),
            ("scan_cloud", "/lidar/points"),
            ("odom", "/ukf_odom/odom"),
        ],
        # arguments=["--ros-args", "--remap", "/rtabmap.rtabmap_viz:=info", "--log-level", "rtabmap_viz:=info"],
        prefix='',
    )
    
    # RViz 노드
    config_rviz = os.path.join(get_package_share_directory('rtabmap_launch'),
                                 'launch', 'config', 'test.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[["-d"], [config_rviz]],
    )
    
    # Voxel cloud 노드 (base camera)
    bc_voxel_cloud_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='bc_voxel_cloud_xyzrgb_node',
        output='screen',
        parameters=[{
            "decimation": 1,
            "voxel_size": 0.0,
            "approx_sync": True,
            "approx_sync_max_interval": 0.5,
            "qos": 0,
            "qos_camera_info": 0,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "max_depth": 10.0,
            "min_depth": 0.05,
            "normal_k": 20,
        }],
        remappings=[
            ("rgb/image", "/base_camera_color_sensor/image_raw"),
            ("rgb/camera_info", "/base_camera_color_sensor/camera_info"),
            ("depth/image", "/base_camera_depth_sensor/depth/image_raw"),
            ("depth/camera_info", "/base_camera_depth_sensor/depth/camera_info"),
            ("rgbd_image", "/base_camera/rgbd_image"),
            ("cloud", "/base_camera/voxel_cloud")
        ]
    )
    
    # Voxel cloud 노드 (gripper camera)
    gc_voxel_cloud_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='gc_voxel_cloud_xyzrgb_node',
        output='screen',
        parameters=[{
            "decimation": 1,
            "voxel_size": 0.0,
            "approx_sync": True,
            "approx_sync_max_interval": 0.5,
            "qos": 0,
            "qos_camera_info": 0,
            "topic_queue_size": 100,
            "sync_queue_size": 100,
            "max_depth": 10.0,
            "min_depth": 0.05,
            "normal_k": 20,
        }],
        remappings=[
            ("rgb/image", "/gripper_camera_color_sensor/image_raw"),
            ("rgb/camera_info", "/gripper_camera_color_sensor/camera_info"),
            ("depth/image", "/gripper_camera_depth_sensor/depth/image_raw"),
            ("depth/camera_info", "/gripper_camera_depth_sensor/depth/camera_info"),
            ("rgbd_image", "/gripper_camera/rgbd_image"),
            ("cloud", "/gripper_camera/voxel_cloud")
        ]
    )
    
    return [
        SetParameter(name='use_sim_time', value='true'),
        gazebo_sim_launch,
        bc_rgbd_sync_node,
        gc_rgbd_sync_node,
        bc_odometry_node,
        gc_odometry_node,
        icp_odometry_node,
        ukf_node,
        slam_node_delayed,
        rtabmap_viz_node,
        rviz_node,
        bc_voxel_cloud_node,
        gc_voxel_cloud_node
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
