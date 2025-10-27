cmake_minimum_required(VERSION 3.8)
project(workflow_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Service-Dateien generieren
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckPartPresence.srv"
  "srv/CheckLabelPresence.srv"
  "srv/GetGraspPoint.srv"
  "srv/MoveToPosition.srv"
  # Weitere .srv oder .msg Dateien hier hinzufügen
)

# Install Python scripts
install(PROGRAMS
  scripts/controller_node.py
  scripts/check_part_presence_server.py
  scripts/check_label_presence_server.py
  scripts/get_grasp_point_server.py
  scripts/move_to_position_server.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()