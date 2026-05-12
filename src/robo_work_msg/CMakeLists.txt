cmake_minimum_required(VERSION 3.8)
project(robo_work_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/IsPartPresent.srv"
  "srv/IsLabelCorrect.srv"
  "srv/GetGraspPoint.srv"  
)

ament_package()