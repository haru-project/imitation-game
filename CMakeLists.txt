cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(imitation_game)

find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      behavior_tree_core
      behavior_tree_ros
      picture_server
)

catkin_package(
)
 catkin_install_python(
   PROGRAMS
   scripts/nlp_yesno_runner.sh
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
add_library(${PROJECT_NAME}
   src/plugin.cpp
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY trees/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trees)
install(DIRECTORY routines/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/routines)
#install(DIRECTORY rviz/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Werror -Wfatal-errors -Wl,--no-undefined)
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES palette_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY palettes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/palettes)
 
 