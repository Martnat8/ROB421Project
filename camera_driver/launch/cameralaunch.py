# The launch file for camera_driver
#
# cameralaunch.py
#
# Nathan Martin


# We need to import the launch system modules.  There's a generic launch
# system, in launch, and some ROS-specific stuff, in launch_ros.
import launch
import launch_ros.actions


# You need to define this function, which is loaded by the launch system.  The function
# returns a list of nodes that you want to run.
def generate_launch_description():
    return launch.LaunchDescription([

        # Runs the oscope nodes
        launch_ros.actions.Node(
            package='camera_driver',
            executable='camera_driver',
            parameters= [{
                'fps': 15.0,
                'encoding': 'bgr8',
                'frame_width': 1280,
                'frame_height': 720,
                'device_id': 0,
                'frame_id' : 'camera',
            }],          
        ),
        # Mediapipe pose estimator
        launch_ros.actions.Node(
            package='mediapipe_pose',
            executable='pose_estimator',
            name='pose_estimator',
        ),
        # Runs
        launch_ros.actions.Node(
            package='angle_extractor',
            executable='angle_publisher',
            name='angle_publisher',
            output='screen',
            # no parameters or remappings needed if you use the same defaults
        ),
        launch_ros.actions.Node(
            package='angle_extractor',
            executable='angle_corrector',
            name='angle_publisher',
            output='screen',

        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
        ])
