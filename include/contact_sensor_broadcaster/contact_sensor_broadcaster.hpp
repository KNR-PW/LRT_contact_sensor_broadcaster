// Copyright (c) 2025, Koło Naukowe Robotyków
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

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef CONTACT_SENSOR_BROADCASTER__CONTACT_SENSOR_BROADCASTER_HPP_
#define CONTACT_SENSOR_BROADCASTER__CONTACT_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "contact_sensor_broadcaster/contact_sensor_broadcaster_parameters.hpp"
#include "contact_sensor_broadcaster/visibility_control.h"
#include "contact_msgs/msg/contact.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "contact_sensor_broadcaster/semantic_component/contact_sensor.hpp"

namespace contact_sensor_broadcaster
{
class ContactSensorBroadcaster : public controller_interface::ControllerInterface
{
public:
  CONTACT_SENSOR_BROADCASTER_PUBLIC
  ContactSensorBroadcaster();

  CONTACT_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CONTACT_SENSOR_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTACT_SENSOR_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

  CONTACT_SENSOR_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTACT_SENSOR_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTACT_SENSOR_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTACT_SENSOR_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  size_t sensor_number_;

  using StatePublisher = rclcpp::Publisher<contact_msgs::msg::Contact>;
  using RealTimeStatePublisher = realtime_tools::RealtimePublisher<contact_msgs::msg::Contact>;
  using ContactSensor = std::unique_ptr<semantic_components::ContactSensor>;
  
  std::vector<ContactSensor> contact_sensors_;
  std::vector<StatePublisher::SharedPtr> sensor_state_publishers_;
  std::vector<std::unique_ptr<RealTimeStatePublisher>> realtime_publishers_;
};

}  // namespace contact_sensor_broadcaster

#endif  // CONTACT_SENSOR_BROADCASTER__CONTACT_SENSOR_BROADCASTER_HPP_