cmake_minimum_required(VERSION 2.8.3)
project(temoto_output_manager)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  temoto_core
  temoto_er_manager
  temoto_robot_manager
  roscpp
  std_msgs
  message_generation
  message_runtime
  rviz_plugin_manager
  cmake_modules
)

add_service_files(FILES
  LoadRvizPlugin.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  temoto_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    cmake_modules
    rviz_plugin_manager 
    temoto_core 
    temoto_er_manager
    temoto_robot_manager
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_output_manager 
  src/temoto_output_manager.cpp
  src/rviz_manager/rviz_manager.cpp
  src/rviz_manager/plugin_info.cpp
)
add_dependencies(temoto_output_manager ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(temoto_output_manager ${catkin_LIBRARIES} temoto_core_components)