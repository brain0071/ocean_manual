cmake_minimum_required(VERSION 3.0.2)
project(ocean_manual)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros_msgs
)

# naodai
find_package(mavlink REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocean_manual
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # naodai
  /home/naodai/WorkSpace/oceanxer/mav_ws/src/mavros
  /usr/include/eigen3
)


# naodai
add_executable(run_manual src/run_manual.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(run_manual
   ${catkin_LIBRARIES}
)



