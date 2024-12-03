import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    world = LaunchConfiguration('world')
    package_dir = get_package_share_directory("turtlebot_bringup")
    robot_description_path = os.path.join(package_dir, 'urdf', 'sample.urdf')

    
    
    # Webots simulator
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'empty_turtle.wbt']),
        mode='realtime',
        ros2_supervisor=True
    )
    
        
    # Define your URDF robots here
    # The name of an URDF robot has to match the WEBOTS_CONTROLLER_URL of the driver node
    # You can specify the URDF file to use with "urdf_path"
    spawn_URDF_robot = URDFSpawner(
        name='myRobot',
        urdf_path=robot_description_path,
        translation='0 0 0.3',
        rotation='0 0 1 0',
    )

    
    robot_driver = WebotsController(
        robot_name='myRobot',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': True,
             'set_robot_state_publisher': True,
             'update_rate': 50
            },
        ],
        respawn=True
    )
    
    

    return LaunchDescription([
        # Starts Webots
        webots,

        # Starts the Ros2Supervisor node created with the WebotsLauncher
        webots._supervisor,

        # Request the spawn of your URDF robot
        spawn_URDF_robot,

        # Launch the driver node once the URDF robot is spawned
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=spawn_URDF_robot,
                on_stdout=lambda event: get_webots_driver_node(event, robot_driver),
            )
        ),

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
