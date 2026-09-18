################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(turtlebot3_isaacsim)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(DIRECTORY launch maps models runtime worlds
  DESTINATION share/${PROJECT_NAME}/
)

# USE_SOURCE_PERMISSIONS: install(DIRECTORY) otherwise applies 644 and strips
# the execute bit off scripts/build_models.sh.
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# A real ROS 2 executable, not installed data: it runs on the system Python
# alongside the simulator, the way turtlebot3_node runs alongside the hardware.
#   ros2 run turtlebot3_isaacsim wheel_odometry
install(PROGRAMS nodes/wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_odometry
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
