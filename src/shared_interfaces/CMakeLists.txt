cmake_minimum_required(VERSION 3.8)
project(shared_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

install(DIRECTORY include/
  DESTINATION include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Direction.msg"
  "msg/Actions.msg"
  "msg/MotorVelocities.msg"
  "msg/DirectionList.msg"
  "msg/ActionsList.msg"
  "msg/MotorVelocitiesList.msg"
  "msg/RobotController.msg"
  "msg/JoyList.msg"
  "msg/BallState.msg"
  "msg/RobotState.msg"
  "msg/GameData.msg"
  "msg/AiDebug.msg"
  "msg/GameStatus.msg"
  "msg/HighScore.msg"
  "msg/HighScoreList.msg"
  DEPENDENCIES builtin_interfaces sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

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