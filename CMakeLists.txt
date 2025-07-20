cmake_minimum_required(VERSION 3.8)
project(contact_sensors_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  contact_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(contact_sensors_broadcaster_parameters
  src/contact_sensors_broadcaster_parameters.yaml
)

add_library(contact_sensors_broadcaster SHARED
  src/contact_sensors_broadcaster.cpp
)

target_compile_features(contact_sensors_broadcaster PUBLIC cxx_std_17)
target_include_directories(contact_sensors_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/contact_sensors_broadcaster>
)
target_link_libraries(contact_sensors_broadcaster PUBLIC
  contact_sensors_broadcaster_parameters
)
ament_target_dependencies(contact_sensors_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(contact_sensors_broadcaster PRIVATE "contact_sensors_broadcaster_BUILDING_DLL")


pluginlib_export_plugin_description_file(
  controller_interface contact_sensors_broadcaster.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include/contact_sensors_broadcaster
)
install(
  TARGETS
    contact_sensors_broadcaster
    contact_sensors_broadcaster_parameters
  EXPORT export_contact_sensors_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_contact_sensors_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
