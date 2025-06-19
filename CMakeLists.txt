cmake_minimum_required(VERSION 3.8)
project(mclaren_robo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(wall_alert_node src/wall_checker.cpp)
ament_target_dependencies(wall_alert_node rclcpp sensor_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install folders properly
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY world/
  DESTINATION share/${PROJECT_NAME}/world
)

install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include
)

install(TARGETS wall_alert_node
  DESTINATION lib/${PROJECT_NAME}
)
# Package export
ament_package()

