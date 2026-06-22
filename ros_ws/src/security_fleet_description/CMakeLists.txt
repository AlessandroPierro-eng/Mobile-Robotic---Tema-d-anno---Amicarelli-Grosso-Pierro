cmake_minimum_required(VERSION 3.8)
project(security_fleet_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Questo è il comando vitale: dice a ROS di copiare la tua cartella 'urdf'
# nei file di sistema quando lanci colcon build!
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()