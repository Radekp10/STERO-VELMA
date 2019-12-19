cmake_minimum_required(VERSION 2.8)
project(path_tf_vis)

find_package(catkin REQUIRED COMPONENTS
   roscpp
  )

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

catkin_install_python(PROGRAMS
    scripts/path_tf_vis
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


