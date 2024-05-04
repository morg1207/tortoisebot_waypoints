cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tortoisebot_waypoints REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)


## ADD LIBRARY
include_directories(include/tortoisebot_waypoints)
set(HEADER_FILES include/tortoisebot_waypoints/tortoisebot_waypoint.hpp )
add_library(tortoisebot_action_lib src/tortoisebot_waypoint.cpp ${HEADER_FILES}) 

ament_target_dependencies(tortoisebot_action_lib rclcpp rclcpp_action geometry_msgs nav_msgs tortoisebot_waypoints tf2)

# this line to export the targets. In your case, it is a library
ament_export_targets(tortoisebot_action_lib HAS_LIBRARY_TARGET)


install(
  DIRECTORY include/tortoisebot_waypoints
  DESTINATION include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES geometry_msgs 
)

add_executable(tortoisebot_action_server_node src/tortoisebot_action_server.cpp)
ament_target_dependencies(tortoisebot_action_server_node rclcpp rclcpp_action geometry_msgs nav_msgs tortoisebot_waypoints tf2)

add_executable(tortoisebot_action_client_node src/tortoisebot_action_client.cpp)
ament_target_dependencies(tortoisebot_action_client_node rclcpp rclcpp_action geometry_msgs tortoisebot_waypoints)




## ADD EXECUTABLE
add_executable(main src/main.cpp)
target_link_libraries(main PUBLIC tortoisebot_action_lib)

install(TARGETS tortoisebot_action_lib
  DESTINATION lib/tortoisebot_action_lib)

install(TARGETS
   main
   tortoisebot_action_server_node
   tortoisebot_action_client_node
   DESTINATION lib/${PROJECT_NAME}
 )


install(
  TARGETS tortoisebot_action_lib
  EXPORT tortoisebot_action_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


#rosidl_target_interfaces(tortoisebot_action_server_node
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  
  set(TESTFILES
      test/main.cpp
      test/tortoisebot_waypoints_test.cpp
      )
  # Add gtest executables
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_test tortoisebot_action_lib )

  install(TARGETS
        ${PROJECT_NAME}_test
        # Install the generated test executable file to the path after DESTINATION
        DESTINATION lib/${PROJECT_NAME})
  # uncomment to enforce lintners and static code analyzers defined in ament_lint_common package        
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
