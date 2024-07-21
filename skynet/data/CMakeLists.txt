cmake_minimum_required(VERSION 3.8)
project(skynet)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR} 
)
# add_subdirectory((deps/crazyflie_tools))

# include_directories(
#   deps/crazyflie_tools/crazyflie_cpp/include
#   deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/include
# #  externalDependencies/libobjecttracker/include
# #  externalDependencies/libmotioncapture/include
#   ${EIGEN3_INCLUDE_DIRS}
# #  ${PCL_INCLUDE_DIRS}
# )

add_executable(teleop src/teleop.cpp)

ament_target_dependencies(teleop
  rclcpp
  std_srvs
  sensor_msgs
  crazyflie_interfaces
  geometry_msgs
)

# add_executable(crazyflie_server 
# src/crazyflie_server.cpp
# )
# target_link_libraries(crazyflie_server
# crazyflie_cpp
# )
# ament_target_dependencies(crazyflie_server
# rclcpp
# tf2_ros
# sensor_msgs
# std_srvs
# crazyflie_interfaces
# motion_capture_tracking_interfaces
# )

# Install C++ executables
install(TARGETS
  # crazyflie_tools
  # comCheck
  # scan
  # listParams
  # listLogVariables
  # listMemories
  # reboot
  # battery
  # version
  # console
  # log
  # setParam
  # downloadUSDLogfile
  # #
  teleop
  # crazyflie_server
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(scripts)

# Install Python executables
install(PROGRAMS
scripts/crazyflie_server.py  
scripts/aideck_streamer.py
scripts/simple_mapper_multiranger.py
scripts/vel_mux.py
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  scripts
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()