// Copyright (c) 2026 Okan Demir
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

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/plugins/action/navigate_through_poses_action.hpp"
#include "nav2_bt_navigator/bt_navigator.hpp"

using namespace std::chrono_literals;

// Requests a goal update during the first goal is running
class GoalUpdateDuringNavigateThroughPoses : public nav2_behavior_tree::NavigateThroughPosesAction
{
public:
  using nav2_behavior_tree::NavigateThroughPosesAction::NavigateThroughPosesAction;

  void on_wait_for_result(
    std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback>/*feedback*/) override
  {
    goal_updated_ = (++wait_count == 1);
  }

  static inline int wait_count = 0;
};

TEST(NavigateThroughPosesGoalUpdate, GoalUpdateLogsMutexError)
{
  // Default no-op tree
  const auto bt_xml = std::filesystem::temp_directory_path() / "navigate_through_poses_tmp.xml";
  std::ofstream(bt_xml) <<
    R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree"><Sleep msec="60000"/></BehaviorTree></root>)";

  // The robot frame is the global frame, so goals in "map" are accepted without TF
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    {"navigators", std::vector<std::string>{"navigate_through_poses"}},
    {"navigate_through_poses.plugin", "nav2_bt_navigator::NavigateThroughPosesNavigator"},
    {"default_nav_through_poses_bt_xml", bt_xml.string()},
    {"robot_base_frame", "map"},
  });
  auto navigator = std::make_shared<nav2_bt_navigator::BtNavigator>(options);
  navigator->configure();
  navigator->activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(navigator->get_node_base_interface());
  std::thread executor_thread([&executor]() {executor.spin();});

  auto blackboard = BT::Blackboard::create();
  blackboard->set<nav2::LifecycleNode::SharedPtr>("node", navigator);
  blackboard->set<std::chrono::milliseconds>("server_timeout", 1000ms);
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
  blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 5000ms);
  blackboard->set<std::chrono::milliseconds>("cancel_timeout", 1000ms);
  blackboard->set("goals", nav_msgs::msg::Goals());

  BT::BehaviorTreeFactory factory;
  factory.registerBuilder<GoalUpdateDuringNavigateThroughPoses>(
    "GoalUpdateDuringNavigateThroughPoses",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<GoalUpdateDuringNavigateThroughPoses>(
        name, "navigate_through_poses", config);
    });
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="MainTree">
         <GoalUpdateDuringNavigateThroughPoses goals="{goals}"/>
       </BehaviorTree></root>)",
    blackboard);

  // Ticks the client until error occurs
  testing::internal::CaptureStderr();
  while (GoalUpdateDuringNavigateThroughPoses::wait_count < 2 &&
    tree.tickOnce() == BT::NodeStatus::RUNNING)
  {
    std::this_thread::sleep_for(10ms);
  }
  const std::string log = testing::internal::GetCapturedStderr();
  EXPECT_EQ(log.find("Navigation requested while another"), std::string::npos);

  tree.haltTree();
  navigator->deactivate();
  navigator->cleanup();
  executor.cancel();
  executor_thread.join();
  std::filesystem::remove(bt_xml);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
