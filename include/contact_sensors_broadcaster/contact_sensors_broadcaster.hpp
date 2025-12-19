// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef CONTACT_SENSORS_BROADCASTER__CONTACT_SENSORS_BROADCASTER_HPP_
#define CONTACT_SENSORS_BROADCASTER__CONTACT_SENSORS_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "contact_sensors_broadcaster/contact_sensors_broadcaster_parameters.hpp"
#include "contact_sensors_broadcaster/visibility_control.h"
#include "contact_msgs/msg/contacts.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "contact_sensors_broadcaster/semantic_component/contact_sensor.hpp"

namespace contact_sensors_broadcaster
{
class ContactSensorsBroadcaster : public controller_interface::ControllerInterface
{
public:
  CONTACT_SENSORS_BROADCASTER_PUBLIC
  ContactSensorsBroadcaster();

  CONTACT_SENSORS_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  CONTACT_SENSORS_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CONTACT_SENSORS_BROADCASTER_PUBLIC controller_interface::CallbackReturn on_init() override;

  CONTACT_SENSORS_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTACT_SENSORS_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTACT_SENSORS_BROADCASTER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CONTACT_SENSORS_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  size_t sensor_number_;

  using StatePublisher = rclcpp::Publisher<contact_msgs::msg::Contacts>;
  using RealTimeStatePublisher = realtime_tools::RealtimePublisher<contact_msgs::msg::Contacts>;
  using ContactSensor = std::unique_ptr<semantic_components::ContactSensor>;
  
  std::vector<ContactSensor> contact_sensors_;
  StatePublisher::SharedPtr sensor_state_publisher_;
  std::unique_ptr<RealTimeStatePublisher> realtime_publisher_;
};

}  // namespace contact_sensors_broadcaster

#endif  // CONTACT_SENSORS_BROADCASTER__CONTACT_SENSORS_BROADCASTER_HPP_