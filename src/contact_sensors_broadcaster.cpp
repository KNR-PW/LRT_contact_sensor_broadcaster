#include "contact_sensors_broadcaster/contact_sensors_broadcaster.hpp"


namespace contact_sensors_broadcaster
{
ContactSensorsBroadcaster::ContactSensorsBroadcaster()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn ContactSensorsBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ContactSensorsBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  
  if (params_.interface_postfix.empty())
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'interface_postfix' not defined, name 'contact' is used.");
  }

  if(params_.publish_with_change)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "'publish_with_change' is 'true', message published only "
      "when one of contact states is changed.");
  }

  if (params_.sensor_names.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'sensor_names' needs to be defined!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.frame_ids.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'frame_ids' needs to be defined!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.sensor_names.size() != params_.frame_ids.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "both 'sensor_name' and "
      "'frame_ids' parameters needs to have same size!");
    return controller_interface::CallbackReturn::ERROR;
  }

  sensor_number_ = params_.sensor_names.size();

  for(size_t i = 0; i < sensor_number_; ++i)
  {
    ContactSensor sensor_ptr = std::make_unique<semantic_components::ContactSensor>(
      semantic_components::ContactSensor(params_.sensor_names[i], params_.interface_postfix));
    contact_sensors_.push_back(std::move(sensor_ptr));
  }

  try
  {
    // register contact sensor data publisher
    sensor_state_publisher_ = get_node()->create_publisher<contact_msgs::msg::Contacts>(
      "~/contact_states", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<RealTimeStatePublisher>(sensor_state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  realtime_publisher_->lock();
  realtime_publisher_->msg_.contacts.resize(sensor_number_);
  for(size_t i = 0; i < sensor_number_; ++i)
  {
    realtime_publisher_->msg_.contacts[i].header.frame_id = params_.frame_ids[i];
  }
  realtime_publisher_->unlock();

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ContactSensorsBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ContactSensorsBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(const auto& contact_sensor: contact_sensors_)
  {
    const std::vector<std::string> & state_interface_names = contact_sensor->get_state_interface_names();
    for(const auto & state_interface_name: state_interface_names)
    {
      state_interfaces_config.names.push_back(state_interface_name);
    }
  }
  return state_interfaces_config;
}

controller_interface::CallbackReturn ContactSensorsBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for(const auto& contact_sensor: contact_sensors_)
  {
    contact_sensor->assign_loaned_state_interfaces(state_interfaces_);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ContactSensorsBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for(const auto& contact_sensor: contact_sensors_)
  {
    contact_sensor->release_interfaces();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ContactSensorsBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    bool changedState = false;
    for(size_t i = 0; i < sensor_number_; ++i)
    {
      bool newFlag = contact_sensors_[i]->get_contact_flag();

      // Check if any flags changed 
      if(realtime_publisher_->msg_.contacts[i].contact != newFlag && !changedState)
      {
        changedState = true;
      }

      realtime_publisher_->msg_.contacts[i].contact = newFlag;
      realtime_publisher_->msg_.contacts[i].header.stamp = time;
    }

    if(params_.publish_with_change)
    {
      if(changedState)
      {
        realtime_publisher_->unlockAndPublish();
      }
      else
      {
        realtime_publisher_->unlock();
      }
    }
    else
    {
      realtime_publisher_->unlockAndPublish();
    }
  }
  
  return controller_interface::return_type::OK;
}
}  // namespace contact_sensors_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  contact_sensors_broadcaster::ContactSensorsBroadcaster,
  controller_interface::ControllerInterface)