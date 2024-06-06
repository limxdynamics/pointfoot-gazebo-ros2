/**
 * @file PointFootHWSim.cpp
 *
 * @brief This file defines the PointFootHWSim class, which is responsible for simulating a legged robot.
 *
 * © [2024] LimX Dynamics Technology Co., Ltd. All rights reserved.
 */

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "gazebo/sensors/ImuSensor.hh"
#include "gazebo/sensors/ForceTorqueSensor.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "pointfoot_gazebo/PointFootHWSim.h"

static limxsdk::PointFootSim *pf;

class pointfoot_gazebo::PointFootGazeboSystemPrivate
{
public:
  PointFootGazeboSystemPrivate() = default;

  ~PointFootGazeboSystemPrivate() = default;

  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief Number of sensors.
  size_t n_sensors_;

  /// \brief Gazebo Model Ptr.
  gazebo::physics::ModelPtr parent_model_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<std::string> joint_names_;

  /// \brief vector with the control method defined in the URDF for each joint.
  std::vector<gazebo_ros2_control::GazeboSystemInterface::ControlMethod> joint_control_methods_;

  /// \brief handles to the joints from within Gazebo
  std::vector<gazebo::physics::JointPtr> sim_joints_;

  /// \brief vector with the current joint position
  std::vector<double> joint_position_;

  /// \brief vector with the current joint velocity
  std::vector<double> joint_velocity_;

  /// \brief vector with the current joint effort
  std::vector<double> joint_effort_;

  /// \brief vector with the current cmd joint position
  std::vector<double> joint_position_cmd_;

  /// \brief vector with the current cmd joint velocity
  std::vector<double> joint_velocity_cmd_;

  /// \brief vector with the current cmd joint effort
  std::vector<double> joint_effort_cmd_;

  /// \brief vector with the current cmd joint position gain
  std::vector<double> Kp_cmd_;

  /// \brief vector with the current cmd joint velocity gain
  std::vector<double> Kd_cmd_;

  /// \brief handles to the imus from within Gazebo
  std::vector<gazebo::sensors::ImuSensorPtr> sim_imu_sensors_;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::vector<std::array<double, 10>> imu_sensor_data_;

  /// \brief handles to the FT sensors from within Gazebo
  std::vector<gazebo::sensors::ForceTorqueSensorPtr> sim_ft_sensors_;

  /// \brief An array per FT sensor for 3D force and torquee
  std::vector<std::array<double, 6>> ft_sensor_data_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
};

namespace pointfoot_gazebo
{
  bool PointFootHWSim::initSim(
      rclcpp::Node::SharedPtr &model_nh,
      gazebo::physics::ModelPtr parent_model,
      const hardware_interface::HardwareInfo &hardware_info,
      sdf::ElementPtr sdf)
  {
    // Initialize the PointFootSim instance and connect to the robot at IP address 127.0.0.1
    pf = limxsdk::PointFootSim::getInstance();
    pf->init("127.0.0.1");

    // Write the number of motors in the robotCmdBuffer_
    robotCmdBuffer_.writeFromNonRT(limxsdk::RobotCmd(pf->getMotorNumber()));

    // Subscribe to robot command messages and write them to robotCmdBuffer_
    pf->subscribeRobotCmdForSim([this](const limxsdk::RobotCmdConstPtr &msg) {
      robotCmdBuffer_.writeFromNonRT(*msg); 
    });

    this->dataPtr_ = std::make_unique<PointFootGazeboSystemPrivate>();
    this->dataPtr_->last_update_sim_time_ros_ = rclcpp::Time();

    this->nh_ = model_nh;
    this->dataPtr_->parent_model_ = parent_model;

    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();

    std::string physics_type_ = physics->GetType();
    if (physics_type_.empty())
    {
      RCLCPP_ERROR(this->nh_->get_logger(), "No physics engine configured in Gazebo.");
      return false;
    }

    registerJoints(hardware_info, parent_model);
    registerSensors(hardware_info, parent_model);

    if (this->dataPtr_->n_dof_ == 0 && this->dataPtr_->n_sensors_ == 0)
    {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), "There is no joint or sensor available");
      return false;
    }

    return true;
  }

  void PointFootHWSim::registerJoints(
      const hardware_interface::HardwareInfo &hardware_info,
      gazebo::physics::ModelPtr parent_model)
  {
    this->dataPtr_->n_dof_ = hardware_info.joints.size();

    this->dataPtr_->joint_names_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->joint_control_methods_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->joint_position_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->joint_velocity_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->joint_effort_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->joint_position_cmd_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->joint_velocity_cmd_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->joint_effort_cmd_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->Kp_cmd_.resize(this->dataPtr_->n_dof_);
    this->dataPtr_->Kd_cmd_.resize(this->dataPtr_->n_dof_);
    std::fill(this->dataPtr_->joint_velocity_.begin(), this->dataPtr_->joint_velocity_.end(), 0.0);

    for (unsigned int j = 0; j < this->dataPtr_->n_dof_; j++)
    {
      std::string joint_name = this->dataPtr_->joint_names_[j] = hardware_info.joints[j].name;

      gazebo::physics::JointPtr simjoint = parent_model->GetJoint(joint_name);
      this->dataPtr_->sim_joints_.push_back(simjoint);
      if (!simjoint)
      {
        RCLCPP_WARN_STREAM(
            this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name << "' which is not in the gazebo model.");
        continue;
      }

      // Accept this joint and continue configuration
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

      // register the command handles
      for (unsigned int i = 0; i < hardware_info.joints[j].command_interfaces.size(); i++)
      {
        if (hardware_info.joints[j].command_interfaces[i].name == "position")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
          this->dataPtr_->joint_control_methods_[j] |= POSITION;
          this->dataPtr_->command_interfaces_.emplace_back(
              joint_name,
              hardware_interface::HW_IF_POSITION,
              &this->dataPtr_->joint_position_cmd_[j]);
        }
        if (hardware_info.joints[j].command_interfaces[i].name == "velocity")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
          this->dataPtr_->joint_control_methods_[j] |= VELOCITY;
          this->dataPtr_->command_interfaces_.emplace_back(
              joint_name,
              hardware_interface::HW_IF_VELOCITY,
              &this->dataPtr_->joint_velocity_cmd_[j]);
        }
        if (hardware_info.joints[j].command_interfaces[i].name == "effort")
        {
          this->dataPtr_->joint_control_methods_[j] |= EFFORT;
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
          this->dataPtr_->command_interfaces_.emplace_back(
              joint_name,
              hardware_interface::HW_IF_EFFORT,
              &this->dataPtr_->joint_effort_cmd_[j]);
        }
        if (hardware_info.joints[j].command_interfaces[i].name == "Kp")
        {
          this->dataPtr_->joint_control_methods_[j] |= EFFORT;
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t Kp");
          this->dataPtr_->command_interfaces_.emplace_back(
              joint_name,
              "Kp",
              &this->dataPtr_->Kp_cmd_[j]);
        }
        if (hardware_info.joints[j].command_interfaces[i].name == "Kd")
        {
          this->dataPtr_->joint_control_methods_[j] |= EFFORT;
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t Kd");
          this->dataPtr_->command_interfaces_.emplace_back(
              joint_name,
              "Kd",
              &this->dataPtr_->Kd_cmd_[j]);
        }
      }

      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

      // register the state handles
      for (unsigned int i = 0; i < hardware_info.joints[j].state_interfaces.size(); i++)
      {
        if (hardware_info.joints[j].state_interfaces[i].name == "position")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
          this->dataPtr_->state_interfaces_.emplace_back(
              joint_name,
              hardware_interface::HW_IF_POSITION,
              &this->dataPtr_->joint_position_[j]);
        }
        if (hardware_info.joints[j].state_interfaces[i].name == "velocity")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
          this->dataPtr_->state_interfaces_.emplace_back(
              joint_name,
              hardware_interface::HW_IF_VELOCITY,
              &this->dataPtr_->joint_velocity_[j]);
        }
        if (hardware_info.joints[j].state_interfaces[i].name == "effort")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
          this->dataPtr_->state_interfaces_.emplace_back(
              joint_name,
              hardware_interface::HW_IF_EFFORT,
              &this->dataPtr_->joint_effort_[j]);
        }
      }
    }
  }

  void PointFootHWSim::registerSensors(
      const hardware_interface::HardwareInfo &hardware_info,
      gazebo::physics::ModelPtr parent_model)
  {
    // Collect gazebo sensor handles
    size_t n_sensors = hardware_info.sensors.size();
    std::vector<hardware_interface::ComponentInfo> imu_components_;
    std::vector<hardware_interface::ComponentInfo> ft_sensor_components_;

    // This is split in two steps: Count the number and type of sensor and associate the interfaces
    // So we have resize only once the structures where the data will be stored, and we can safely
    // use pointers to the structures
    for (unsigned int j = 0; j < n_sensors; j++)
    {
      hardware_interface::ComponentInfo component = hardware_info.sensors[j];
      std::string sensor_name = component.name;

      // This can't be used, because it only looks for sensor in links, but force_torque_sensor
      // must be in a joint, as in not found by SensorScopedName
      // std::vector<std::string> gz_sensor_names = parent_model->SensorScopedName(sensor_name);

      // Workaround to find sensors whose parent is a link or joint of parent_model
      std::vector<std::string> gz_sensor_names;
      for (const auto &s : gazebo::sensors::SensorManager::Instance()->GetSensors())
      {
        const std::string sensor_parent = s->ParentName();
        if (s->Name() != sensor_name)
        {
          continue;
        }
        if ((parent_model->GetJoint(sensor_parent) != nullptr) ||
            parent_model->GetLink(sensor_parent) != nullptr)
        {
          gz_sensor_names.push_back(s->ScopedName());
        }
      }
      if (gz_sensor_names.empty())
      {
        RCLCPP_WARN_STREAM(
            this->nh_->get_logger(), "Skipping sensor in the URDF named '" << sensor_name << "' which is not in the gazebo model.");
        continue;
      }
      if (gz_sensor_names.size() > 1)
      {
        RCLCPP_WARN_STREAM(
            this->nh_->get_logger(), "Sensor in the URDF named '" << sensor_name << "' has more than one gazebo sensor with the " << "same name, only using the first. It has " << gz_sensor_names.size() << " sensors");
      }

      gazebo::sensors::SensorPtr simsensor = gazebo::sensors::SensorManager::Instance()->GetSensor(
          gz_sensor_names[0]);
      if (!simsensor)
      {
        RCLCPP_ERROR_STREAM(
            this->nh_->get_logger(),
            "Error retrieving sensor '" << sensor_name << " from the sensor manager");
        continue;
      }
      if (simsensor->Type() == "imu")
      {
        gazebo::sensors::ImuSensorPtr imu_sensor =
            std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(simsensor);
        if (!imu_sensor)
        {
          RCLCPP_ERROR_STREAM(
              this->nh_->get_logger(),
              "Error retrieving casting sensor '" << sensor_name << " to ImuSensor");
          continue;
        }
        imu_components_.push_back(component);
        this->dataPtr_->sim_imu_sensors_.push_back(imu_sensor);
      }
      else if (simsensor->Type() == "force_torque")
      {
        gazebo::sensors::ForceTorqueSensorPtr ft_sensor =
            std::dynamic_pointer_cast<gazebo::sensors::ForceTorqueSensor>(simsensor);
        if (!ft_sensor)
        {
          RCLCPP_ERROR_STREAM(
              this->nh_->get_logger(),
              "Error retrieving casting sensor '" << sensor_name << " to ForceTorqueSensor");
          continue;
        }
        ft_sensor_components_.push_back(component);
        this->dataPtr_->sim_ft_sensors_.push_back(ft_sensor);
      }
    }

    this->dataPtr_->imu_sensor_data_.resize(this->dataPtr_->sim_imu_sensors_.size());
    this->dataPtr_->ft_sensor_data_.resize(this->dataPtr_->sim_ft_sensors_.size());
    this->dataPtr_->n_sensors_ = this->dataPtr_->sim_imu_sensors_.size() +
                                 this->dataPtr_->sim_ft_sensors_.size();

    for (unsigned int i = 0; i < imu_components_.size(); i++)
    {
      const std::string &sensor_name = imu_components_[i].name;
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << sensor_name);
      RCLCPP_INFO_STREAM(
          this->nh_->get_logger(), "\tState:");
      for (const auto &state_interface : imu_components_[i].state_interfaces)
      {
        static const std::map<std::string, size_t> interface_name_map = {
            {"orientation.x", 0},
            {"orientation.y", 1},
            {"orientation.z", 2},
            {"orientation.w", 3},
            {"angular_velocity.x", 4},
            {"angular_velocity.y", 5},
            {"angular_velocity.z", 6},
            {"linear_acceleration.x", 7},
            {"linear_acceleration.y", 8},
            {"linear_acceleration.z", 9},
        };
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

        size_t data_index = interface_name_map.at(state_interface.name);
        this->dataPtr_->state_interfaces_.emplace_back(
            sensor_name,
            state_interface.name,
            &this->dataPtr_->imu_sensor_data_[i][data_index]);
      }
    }
    for (unsigned int i = 0; i < ft_sensor_components_.size(); i++)
    {
      const std::string &sensor_name = ft_sensor_components_[i].name;
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << sensor_name);
      RCLCPP_INFO_STREAM(
          this->nh_->get_logger(), "\tState:");
      for (const auto &state_interface : ft_sensor_components_[i].state_interfaces)
      {
        static const std::map<std::string, size_t> interface_name_map = {
            {"force.x", 0},
            {"force.y", 1},
            {"force.z", 2},
            {"torque.x", 3},
            {"torque.y", 4},
            {"torque.z", 5}};
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

        size_t data_index = interface_name_map.at(state_interface.name);
        this->dataPtr_->state_interfaces_.emplace_back(
            sensor_name,
            state_interface.name,
            &this->dataPtr_->ft_sensor_data_[i][data_index]);
      }
    }
  }

  hardware_interface::return_type
  PointFootHWSim::configure(const hardware_interface::HardwareInfo &actuator_info)
  {
    if (configure_default(actuator_info) != hardware_interface::return_type::OK)
    {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface>
  PointFootHWSim::export_state_interfaces()
  {
    return std::move(this->dataPtr_->state_interfaces_);
  }

  std::vector<hardware_interface::CommandInterface>
  PointFootHWSim::export_command_interfaces()
  {
    return std::move(this->dataPtr_->command_interfaces_);
  }

  hardware_interface::return_type PointFootHWSim::start()
  {
    status_ = hardware_interface::status::STARTED;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type PointFootHWSim::stop()
  {
    status_ = hardware_interface::status::STOPPED;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type PointFootHWSim::read()
  {
    const gazebo::common::Time gz_time_now = this->dataPtr_->parent_model_->GetWorld()->SimTime();
    const rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    const rclcpp::Duration sim_period = sim_time_ros - this->dataPtr_->last_update_sim_time_ros_;

    for (unsigned int j = 0; j < this->dataPtr_->joint_names_.size(); j++)
    {
      if (this->dataPtr_->sim_joints_[j])
      {
        this->dataPtr_->joint_position_[j] = this->dataPtr_->sim_joints_[j]->Position(0);
        this->dataPtr_->joint_velocity_[j] = this->dataPtr_->sim_joints_[j]->GetVelocity(0);
        this->dataPtr_->joint_effort_[j] = this->dataPtr_->sim_joints_[j]->GetForce(0u);
      }
    }

    for (unsigned int j = 0; j < this->dataPtr_->sim_imu_sensors_.size(); j++)
    {
      auto sim_imu = this->dataPtr_->sim_imu_sensors_[j];
      this->dataPtr_->imu_sensor_data_[j][0] = sim_imu->Orientation().X();
      this->dataPtr_->imu_sensor_data_[j][1] = sim_imu->Orientation().Y();
      this->dataPtr_->imu_sensor_data_[j][2] = sim_imu->Orientation().Z();
      this->dataPtr_->imu_sensor_data_[j][3] = sim_imu->Orientation().W();

      this->dataPtr_->imu_sensor_data_[j][4] = sim_imu->AngularVelocity().X();
      this->dataPtr_->imu_sensor_data_[j][5] = sim_imu->AngularVelocity().Y();
      this->dataPtr_->imu_sensor_data_[j][6] = sim_imu->AngularVelocity().Z();

      this->dataPtr_->imu_sensor_data_[j][7] = sim_imu->LinearAcceleration().X();
      this->dataPtr_->imu_sensor_data_[j][8] = sim_imu->LinearAcceleration().Y();
      this->dataPtr_->imu_sensor_data_[j][9] = sim_imu->LinearAcceleration().Z();
    }

    for (unsigned int j = 0; j < this->dataPtr_->sim_ft_sensors_.size(); j++)
    {
      auto sim_ft_sensor = this->dataPtr_->sim_ft_sensors_[j];
      this->dataPtr_->imu_sensor_data_[j][0] = sim_ft_sensor->Force().X();
      this->dataPtr_->imu_sensor_data_[j][1] = sim_ft_sensor->Force().Y();
      this->dataPtr_->imu_sensor_data_[j][2] = sim_ft_sensor->Force().Z();

      this->dataPtr_->imu_sensor_data_[j][3] = sim_ft_sensor->Torque().X();
      this->dataPtr_->imu_sensor_data_[j][4] = sim_ft_sensor->Torque().Y();
      this->dataPtr_->imu_sensor_data_[j][5] = sim_ft_sensor->Torque().Z();
    }
    
    // Publish robot state for simulation
    if (this->dataPtr_->joint_names_.size() == pf->getMotorNumber())
    {
      limxsdk::RobotState state;
      state.q.resize(this->dataPtr_->joint_names_.size());
      state.dq.resize(this->dataPtr_->joint_names_.size());
      state.tau.resize(this->dataPtr_->joint_names_.size());

      // Fill in joint position, velocity, and effort values
      for (std::size_t i = 0; i < this->dataPtr_->joint_names_.size(); i++)
      {
        int index = parseJointIndex(this->dataPtr_->joint_names_[i]);
        if (index != -1)
        {
          state.stamp = rclcpp::Clock().now().nanoseconds();
          state.q[index] = this->dataPtr_->joint_position_[i];
          state.dq[index] = this->dataPtr_->joint_velocity_[i];
          state.tau[index] = this->dataPtr_->joint_effort_[i];
        }
      }
      pf->publishRobotStateForSim(state);
    }

    // Fill in IMU sensor data
    if (this->dataPtr_->sim_imu_sensors_.size() > 0)
    {
      limxsdk::ImuData imu_data;
      imu_data.stamp = rclcpp::Clock().now().nanoseconds();
      auto &imu = this->dataPtr_->imu_sensor_data_.front();
      imu_data.quat[1] = imu[0]; // X
      imu_data.quat[2] = imu[1]; // Y
      imu_data.quat[3] = imu[2]; // Z
      imu_data.quat[0] = imu[3]; // W
      imu_data.gyro[0] = imu[4]; // X
      imu_data.gyro[1] = imu[5]; // Y
      imu_data.gyro[2] = imu[6]; // Z
      imu_data.acc[0] = imu[7];  // X
      imu_data.acc[1] = imu[8];  // Y
      imu_data.acc[2] = imu[9];  // Z
      pf->publishImuDataForSim(imu_data);
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type PointFootHWSim::write()
  {
    // Get the simulation time and period
    gazebo::common::Time gz_time_now = this->dataPtr_->parent_model_->GetWorld()->SimTime();
    rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    rclcpp::Duration sim_period = sim_time_ros - this->dataPtr_->last_update_sim_time_ros_;

    if (this->dataPtr_->joint_names_.size() == pf->getMotorNumber())
    {
      // Update joint data with commands from robot command buffer
      auto robotcmd = *robotCmdBuffer_.readFromRT();

      for (unsigned int j = 0; j < this->dataPtr_->joint_names_.size(); j++)
      {
        if (this->dataPtr_->sim_joints_[j])
        {
          int index = parseJointIndex(this->dataPtr_->joint_names_[j]);
          this->dataPtr_->joint_effort_cmd_[j] = robotcmd.tau[index];
          this->dataPtr_->joint_position_cmd_[j] = robotcmd.q[index];
          this->dataPtr_->joint_velocity_cmd_[j] = robotcmd.dq[index];
          this->dataPtr_->Kp_cmd_[j] = robotcmd.Kp[index];
          this->dataPtr_->Kd_cmd_[j] = robotcmd.Kd[index];
          
          const double tau_feedforward = this->dataPtr_->joint_effort_cmd_[j];
          const double pos_feedback =
              this->dataPtr_->Kp_cmd_[j] * (this->dataPtr_->joint_position_cmd_[j] - this->dataPtr_->joint_position_[j]);
          const double vel_feedback =
              this->dataPtr_->Kd_cmd_[j] * (this->dataPtr_->joint_velocity_cmd_[j] - this->dataPtr_->joint_velocity_[j]);
          const double effort = tau_feedforward + pos_feedback + vel_feedback;
          this->dataPtr_->sim_joints_[j]->SetForce(0, effort);
        }
      }
    }

    this->dataPtr_->last_update_sim_time_ros_ = sim_time_ros;

    return hardware_interface::return_type::OK;
  }

  int PointFootHWSim::parseJointIndex(const std::string &jointName)
  {
    int leg_index = 0;
    int joint_index = 0;

    if (jointName.find("L") != std::string::npos)
    {
      leg_index = 0;
    }
    else if (jointName.find("R") != std::string::npos)
    {
      leg_index = 1;
    }
    else
    {
      return -1;
    }

    if (jointName.find("abad") != std::string::npos)
    {
      joint_index = 0;
    }
    else if (jointName.find("hip") != std::string::npos)
    {
      joint_index = 1;
    }
    else if (jointName.find("knee") != std::string::npos)
    {
      joint_index = 2;
    }
    else
    {
      return -1;
    }

    return (leg_index * 3 + joint_index);
  }

} // namespace pointfoot_gazebo

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pointfoot_gazebo::PointFootHWSim, gazebo_ros2_control::GazeboSystemInterface)
