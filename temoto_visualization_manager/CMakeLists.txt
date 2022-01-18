cmake_minimum_required(VERSION 2.8.3)
project(temoto_visualization_manager)

add_compile_options(-std=c++14)

option(TEMOTO_ENABLE_TRACING_ "Use tracer" OFF)
if(TEMOTO_ENABLE_TRACING)
  add_compile_options(-Denable_tracing)
endif()

find_package(catkin REQUIRED
  temoto_resource_registrar
  temoto_er_manager
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
  temoto_resource_registrar
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    cmake_modules
    rviz_plugin_manager 
    temoto_resource_registrar
    temoto_er_manager
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_visualization_manager 
  src/visualization_manager_node.cpp
  src/visualization_manager.cpp
  src/plugin_info.cpp
)
add_dependencies(temoto_visualization_manager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(temoto_visualization_manager
  ${catkin_LIBRARIES}
)

add_executable(visualization_manager_test 
  src/examples/visualization_manager_test.cpp
)
add_dependencies(visualization_manager_test
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(visualization_manager_test
  ${catkin_LIBRARIES}
)

install(TARGETS temoto_visualization_manager
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)