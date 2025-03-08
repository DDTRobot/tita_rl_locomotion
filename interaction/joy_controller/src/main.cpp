// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <time.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "joy_controller/joy_controller.hpp"

int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<joy_controller::JoyCommand>(options);
  node->init(node);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();

  return 0;
}
