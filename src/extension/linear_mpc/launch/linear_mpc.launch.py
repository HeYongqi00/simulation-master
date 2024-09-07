# Copyright (c) 2021 Geekplus Inc
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    
    config = os.getcwd() + "/src/extension/linear_mpc/config/"+ 'linear_mpc.yaml'
    ld = LaunchDescription()

    linear_mpc = Node(
        package='linear_mpc',
        name='linear_mpc',
        executable='linear_mpc',
        output='screen',
        parameters=[config]
    )
    ld.add_action(linear_mpc)

    return ld
