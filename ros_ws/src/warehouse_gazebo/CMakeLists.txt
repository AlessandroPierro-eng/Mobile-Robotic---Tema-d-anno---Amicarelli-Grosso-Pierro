cmake_minimum_required(VERSION 3.8)
project(warehouse_gazebo)

# Compiler diagnostics
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install shared simulation assets
install(
  DIRECTORY launch worlds map
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(
  PROGRAMS scripts/mock_yolo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()