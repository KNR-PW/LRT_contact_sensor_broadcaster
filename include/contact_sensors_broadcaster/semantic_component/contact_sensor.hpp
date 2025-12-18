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

  /// Constructor for contact sensor with custom interface postfix.
  ContactSensor(const std::string & name, const std::string & interface_postfix)
  : SemanticComponentInterface(name, 1)
  {
    interface_names_.emplace_back(name + "/" + interface_postfix);

    contact_flag_ = false;
  }

  virtual ~ContactSensor() = default;

  /// Return contact flag.
  /**
   * \return contact flag as boolean
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

  /// Return Contact message with contact flag.
  /**
   * Return a contact message with current bollean value.
   *
   * \return contact message with changed boolean (header needs to be changed in boradcaster).
   */
  bool get_values_as_message(contact_msgs::msg::Contact& message)
  {
    // call get_contact_flag() to update with the latest value
    get_contact_flag();

    message.contact = contact_flag_;

    return true;
  }

protected:
  
  /// Boolean that represent it there is or is not contact
  bool contact_flag_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__CONTACT_SENSOR_HPP_