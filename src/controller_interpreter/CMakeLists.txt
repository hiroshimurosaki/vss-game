cmake_minimum_required(VERSION 3.8)
project(controller_interpreter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)
find_package(SDL2 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(joy_aggregator src/JoyAggregator.cpp)
target_include_directories(joy_aggregator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(joy_aggregator PUBLIC cxx_std_17)
ament_target_dependencies(joy_aggregator
  rclcpp
  sensor_msgs
  shared_interfaces)

add_executable(special_controls
  src/SpecialControls.cpp
)

target_include_directories(special_controls PUBLIC
  include
)

ament_target_dependencies(special_controls
  rclcpp
  shared_interfaces
)

add_executable(direction src/DirectionNode.cpp)
target_include_directories(direction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(direction PUBLIC cxx_std_17)
ament_target_dependencies(direction
  rclcpp
  shared_interfaces)

add_executable(keyboard_input src/KeyboardInput.cpp)
target_include_directories(keyboard_input PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${SDL2_INCLUDE_DIRS})
target_compile_features(keyboard_input PUBLIC cxx_std_17)
target_link_libraries(keyboard_input ${SDL2_LIBRARIES})
ament_target_dependencies(keyboard_input
  rclcpp
  sensor_msgs)

install(TARGETS 
  joy_aggregator
  special_controls
  direction
  keyboard_input
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()