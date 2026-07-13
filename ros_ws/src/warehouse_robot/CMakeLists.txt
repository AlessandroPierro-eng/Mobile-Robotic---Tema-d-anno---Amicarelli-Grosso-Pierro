cmake_minimum_required(VERSION 3.8)
project(warehouse_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(rviz2 REQUIRED)
find_package(turtlebot4_description REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(robot_state_publisher REQUIRED)

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

# Installazione delle cartelle di lancio e dei parametri di navigazione
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================================================
# NUOVA SEZIONE AGGIUNTA: INSTALLAZIONE DEGLI SCRIPT PYTHON
# Questo comando indica a CMake di prendere lo script 'yolo_detector.py', 
# di riconoscerlo come un programma eseguibile e di installarlo nella cartella 
# di sistema 'lib/warehouse_robot'. 
# In questo modo, il comando 'ros2 run' saprà esattamente dove andarlo a pescare.
# Quando aggiungerai altri nodi, basterà aggiungerli a questa lista.
# ==============================================================================
install(PROGRAMS
  warehouse_robot/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()