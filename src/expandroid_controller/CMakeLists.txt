cmake_minimum_required(VERSION 3.8)
project(expandroid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(expandroid_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  expandroid_msgs
)

add_executable(expandroid_control_node src/expandroid_control_node.cpp)
target_include_directories(expandroid_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(expandroid_control_node PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

ament_target_dependencies(expandroid_control_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS expandroid_control_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
