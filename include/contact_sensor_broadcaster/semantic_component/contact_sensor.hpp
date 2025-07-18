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

#ifndef SEMANTIC_COMPONENTS__CONTACT_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__CONTACT_SENSOR_HPP_

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "contact_msgs/msg/contact.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace semantic_components
{
class ContactSensor : public SemanticComponentInterface<contact_msgs::msg::Contact>
{
public:
  /// Constructor for "standard" contact sensor
  explicit ContactSensor(const std::string & name) : SemanticComponentInterface(name, 1)
  {
    // If contact sensor use standard names
    interface_names_.emplace_back(name + "/" + "contact");

    contact_flag_ = false;
  }

  /// Constructor for contact sensor with custom interface names.
  /**
   * Constructor for contact sensor with custom interface names or FTS with less then six measurement axes,
   * e.g., 1D and 2D force load cells.
   * For non existing axes interface is empty string, i.e., ("");
   *
   * The name should be in the following order:
   *   force X, force Y, force Z, torque X, torque Y, torque Z.
   */
  ContactSensor(const std::string & name, const std::string & interface_postfix)
  : SemanticComponentInterface(name, 1)
  {
    interface_names_.emplace_back(name + "/" + interface_postfix);

    contact_flag_ = false;
  }

  virtual ~ContactSensor() = default;

  /// Return contact flag.
  /**
   * 
   *
   * \return contact flag as bool
   */
  bool get_contact_flag()
  {
    if(state_interfaces_[0].get().get_value() == 1.0)
    {
      contact_flag_ = true;
    }
    else
    {
      contact_flag_ = false;
    }
    return contact_flag_;
  }

  /// Return Contact message with forces and torques.
  /**
   * Constructs and return a contact message from the current values.
   *
   * \return contact message from values WITHOUT HEADER;
   */
  bool get_values_as_message(contact_msgs::msg::Contact & message)
  {
    // call get_contact_flag() to update with the latest value
    get_contact_flag();

    message.contact = contact_flag_;

    return true;
  }

protected:
  
  /// Bool that represent it there is or is not contact
  bool contact_flag_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__CONTACT_SENSOR_HPP_