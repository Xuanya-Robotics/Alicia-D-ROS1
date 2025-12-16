cmake_minimum_required(VERSION 3.0.2)
project(alicia_d_driver)

# Set the C++ standard to C++17 to support std::optional and other modern features.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find Dependencies ---
# Find all required ROS packages.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  hardware_interface
  controller_manager
)

# LibSerial and zlib are used by the new serial_communicator and parser, like in ROS2 driver
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)
find_package(ZLIB REQUIRED)


# --- Catkin Package Information ---
# This is for other ROS packages that might depend on this one in the future.
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs hardware_interface controller_manager serial
)

# --- Include Directories ---
# Specify all locations of header files.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SERIAL_INCLUDE_DIRS}
)


# --- Build Targets ---

# 1. Hardware Driver Node (communicates with serial port)
add_executable(alicia_d_driver_node
  src/alicia_d_driver_node.cpp
  src/serial_communicator.cpp
  src/alicia_d_data_parser_control.cpp
)
# Link against catkin libraries and LibSerial/ZLIB (for CRC and serial handling)
target_link_libraries(alicia_d_driver_node
  ${catkin_LIBRARIES}
  ${SERIAL_LIBRARIES}
  ZLIB::ZLIB
)

# 2. ros_control Interface Library (the bridge)
add_library(alicia_d_hw_interface src/alicia_d_hw_interface.cpp)
target_include_directories(alicia_d_hw_interface PUBLIC
      ${catkin_INCLUDE_DIRS}
)
target_link_libraries(alicia_d_hw_interface
  ${catkin_LIBRARIES}
)

# 3. ros_control Main Executable (runs the control loop)
add_executable(alicia_d_hw_interface_node src/alicia_d_hw_main.cpp)
target_link_libraries(alicia_d_hw_interface_node
  alicia_d_hw_interface # This executable needs the library we just defined
  ${catkin_LIBRARIES}
)


# --- Installation Rules ---
# This section remains correct.
install(TARGETS
  alicia_d_driver_node
  alicia_d_hw_interface_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  alicia_d_hw_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)