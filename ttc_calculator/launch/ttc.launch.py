from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ttc_calculator',
            executable='ttc_node',
            name='ttc_node',
            output='screen',
            parameters=[{
                # 'pointcloud_topic': '/ouster/points',
                # 'twist_topic': '/vehicle/twist',
                # 'fov_angle_deg': 10.0,
                # 'ground_z_threshold': 0.2,
                # 'min_velocity_threshold': 0.1,
                # 'min_cluster_points': 5,
                # 'cluster_radius': 0.5,
                # 'csv_filename': 'ttc_data.csv',
                # 'yaw_offset_deg': 180.0,   # set 180 if LiDAR frame faces rear
            }],
        )
    ])
