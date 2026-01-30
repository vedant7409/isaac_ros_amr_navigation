cmake_minimum_required(VERSION 3.8)
project(isaac_ros_amr_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  robot_description/
  DESTINATION share/${PROJECT_NAME}/robot_description
)

install(DIRECTORY
  isaac_ros_perception/
  DESTINATION share/${PROJECT_NAME}/isaac_ros_perception
)

install(DIRECTORY
  navigation/
  DESTINATION share/${PROJECT_NAME}/navigation
)

install(DIRECTORY
  gazebo/
  DESTINATION share/${PROJECT_NAME}/gazebo
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
