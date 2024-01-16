cmake_minimum_required(VERSION 2.8.3)
project(improved_local_planner)

set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/DBLocalPlannerReconfigure.cfg
)

catkin_package(
  CATKIN_DEPENDS
	dynamic_reconfigure
)