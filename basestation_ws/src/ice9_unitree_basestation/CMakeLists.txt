cmake_minimum_required(VERSION 3.1)
project(ice9_unitree_basestation)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
