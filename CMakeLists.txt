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
install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}/
)

# USE_SOURCE_PERMISSIONS: install(DIRECTORY) otherwise applies 644 and strips
# the execute bit off scripts/build_models.sh.
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
