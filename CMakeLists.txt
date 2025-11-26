cmake_minimum_required(VERSION 3.16)
project(origin_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Find ament dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# === LibSerial setup ===
# Include directory
include_directories(/usr/include/libserial)
# Find library
find_library(LIBSERIAL_LIB serial PATHS /usr/lib /usr/lib/x86_64-linux-gnu REQUIRED)

# === Compile library ===
add_library(
  origin_hardware
  SHARED
  hardware/origin_system.cpp
)

target_compile_features(origin_hardware PUBLIC cxx_std_17)

# Include directories
target_include_directories(origin_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/origin_hardware>
  /usr/include/libserial
)

# Link libraries
target_link_libraries(origin_hardware PUBLIC ${LIBSERIAL_LIB})

# ROS2 dependencies
ament_target_dependencies(origin_hardware PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ORIGIN_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

# === Install ===
install(
  DIRECTORY hardware/include/
  DESTINATION include/origin_hardware
)

install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/origin_hardware
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/origin_hardware
)

install(TARGETS origin_hardware
  EXPORT export_origin_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# === Testing ===
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# === Exports ===
ament_export_targets(export_origin_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
