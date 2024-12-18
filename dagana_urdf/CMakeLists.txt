project(${ROBOT_NAME}_urdf)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

# mark urdf, meshes and launch files for installation
install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# main urdf
generate_urdf(
  XACRO ${ROBOT_NAME}.urdf.xacro 
  CONFIG config/${ROBOT_NAME}.urdf.xacro 
  ADD_TO_ALL TRUE
)
