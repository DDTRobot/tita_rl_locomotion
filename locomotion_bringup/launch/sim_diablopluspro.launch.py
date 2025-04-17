# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription, LaunchContext
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

prefix = "tita"


def generate_launch_description():
    # Declare an argument to control inclusion of the extra launch file
    declared_arguments = []

    # Declare the simulation environment argument (webots or gazebo)
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_env",
            default_value="gazebo",
            description="Select simulation environment",
            choices=["webots", "gazebo"],
        )
    )

    # Declare the control mode argument (wbc, sdk, or mcu)
    declared_arguments.append(
        DeclareLaunchArgument(
            "ctrl_mode",
            default_value="wbc",
            choices=["wbc", "sdk", "mcu"],
            description="Select wheel-legged robot control methods, mcu means on mcu-board control",
        )
    )

    urdf = "robot.xacro"
    yaml_path = "locomotion_bringup"

    # Include the Webots controller manager launch file if sim_env is webots
    webots_controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("webots_bridge"),
                "launch",
                "webots_bridge.launch.py",
            )
        ),
        launch_arguments={
            "ctrl_mode": LaunchConfiguration("ctrl_mode"),
            "urdf": urdf,
            "yaml_path": yaml_path,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("sim_env"), "' == 'webots'"])
        ),
    )

    # Include the Gazebo controller manager launch file if sim_env is gazebo
    gazebo_controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_bridge"),
                "launch",
                "gazebo_bridge2.launch.py",
            )
        ),
        launch_arguments={
            "ctrl_mode": LaunchConfiguration("ctrl_mode"),
            "urdf": urdf,
            "yaml_path": yaml_path,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("sim_env"), "' == 'gazebo'"])
        ),
    )

    # Include the robot inertia calculator launch file #tofix
    robot_inertia_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_inertia_calculator"),
                "launch",
                "robot_inertia_calculator.launch.py",
            )
        ),
    )

    # Node to spawn the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            prefix + "/controller_manager",
        ],
    )

    # Node to spawn the IMU sensor broadcaster
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            prefix + "/controller_manager",
        ],
    )

    # Node to spawn the WBC controller
    wbc_controller = Node(
        package="controller_manager",
        # output='screen',
        executable="spawner",
        arguments=[
            "tita_controller",
            "--controller-manager",
            prefix + "/controller_manager",
        ],
        # condition=IfCondition(PythonExpression(["'", LaunchConfiguration('ctrl_mode'), "' == 'wbc'"]))
    )

    # Return the launch description with all declared arguments and included nodes
    return LaunchDescription(
        declared_arguments
        + [
            webots_controller_manager_launch,
            gazebo_controller_manager_launch,
            joint_state_broadcaster_spawner,
            imu_sensor_broadcaster_spawner,
            wbc_controller,
            robot_inertia_launch,
        ]
    )
