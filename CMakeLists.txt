cmake_minimum_required(VERSION 3.0.2)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0057 NEW)

project(irsl_choreonoid_ros)

#add_compile_options(-std=c++17)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

# find_package(choreonoid_ros REQUIRED)

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)
