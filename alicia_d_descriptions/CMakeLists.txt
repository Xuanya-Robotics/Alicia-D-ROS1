cmake_minimum_required(VERSION 3.0.2)
project(alicia_d_descriptions)

find_package(catkin REQUIRED)

catkin_package()

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

# Install config directory
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  OPTIONAL
)

# Install scripts
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Make Python scripts and shell scripts executable
install(PROGRAMS
  launch/load_urdf.py
  launch/prepare_moveit_urdf.py
  scripts/setup_moveit.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
