from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gtsam_test',
            executable='pose_estimator_node',
            name='pose_estimator',
            output='screen',
            parameters=[{
                'optimization_period_ms': 500,
                'between_noise_translation': 1.0,
                'between_noise_rotation': 0.1,
                'min_dt_for_new_node': 0.001,
            }],
            remappings=[
                ('/gps/pose_with_covariance', '/gps/pose_with_covariance'),
                ('/lidar/pose_with_covariance', '/lidar/pose_with_covariance'),
            ],
        ),
    ])
