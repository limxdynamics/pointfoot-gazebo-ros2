/**
 * @file PointFootHWSim.h
 *
 * @brief This file defines the PointFootHWSim class, which is responsible for simulating a legged robot.
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"

#include "gazebo_ros2_control/gazebo_system_interface.hpp"
#include <realtime_tools/realtime_buffer.h>
#include "limxsdk/pointfoot_sim.h"

namespace pointfoot_gazebo
{

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Forward declaration
  class PointFootGazeboSystemPrivate;

  // These class must inherit `gazebo_ros2_control::GazeboSystemInterface` which implements a
  // simulated `ros2_control` `hardware_interface::SystemInterface`.

  class PointFootHWSim : public gazebo_ros2_control::GazeboSystemInterface
  {
  public:
    // Documentation Inherited
    CallbackReturn on_init(const hardware_interface::HardwareInfo &system_info)
        override;

    // Documentation Inherited
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    // Documentation Inherited
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Documentation Inherited
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    // Documentation Inherited
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    // Documentation Inherited
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string> &start_interfaces,
        const std::vector<std::string> &stop_interfaces) override;

    // Documentation Inherited
    hardware_interface::return_type read(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

    // Documentation Inherited
    hardware_interface::return_type write(
        const rclcpp::Time &time,
        const rclcpp::Duration &period) override;

    // Documentation Inherited
    bool initSim(
        rclcpp::Node::SharedPtr &model_nh,
        gazebo::physics::ModelPtr parent_model,
        const hardware_interface::HardwareInfo &hardware_info,
        sdf::ElementPtr sdf) override;

  private:
    void registerJoints(
        const hardware_interface::HardwareInfo &hardware_info,
        gazebo::physics::ModelPtr parent_model);

    void registerSensors(
        const hardware_interface::HardwareInfo &hardware_info,
        gazebo::physics::ModelPtr parent_model);

    int parseJointIndex(const std::string &jointName);

    /// \brief Private data class
    std::unique_ptr<PointFootGazeboSystemPrivate> dataPtr_;

    realtime_tools::RealtimeBuffer<limxsdk::RobotCmd> robotCmdBuffer_; // Real-time buffer for robot commands
  };

} // namespace pointfoot_gazebo
