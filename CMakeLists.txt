cmake_minimum_required(VERSION 3.5)

project(turtlesim)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()
set (CMAKE_AUTOMOC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(Qt4 4.6 REQUIRED)
include(${QT_USE_FILE})

include_directories(include)
set(turtlesim_node_HDRS
  include/turtlesim/turtle_frame.hpp
)
qt4_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Color.msg"
  "msg/Pose.msg"
  "srv/Empty.srv"
  "srv/SetPen.srv"
  "srv/TeleportAbsolute.srv"
  "srv/TeleportRelative.srv"
  "srv/Spawn.srv"
  "srv/Kill.srv"
)

ament_export_dependencies(rosidl_default_runtime)
MESSAGE("=================${QT_LIBRARIES}======================")


add_executable(turtlesim_node 
  src/turtlesim.cpp 
  src/turtle_frame.cpp 
  src/turtle.cpp
  ${turtlesim_node_MOCS}
)

get_available_rmw_implementations(rmw_implementations2)
foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(turtlesim_node
      ${PROJECT_NAME} ${typesupport_impl}
    )
  endforeach()
endforeach()

target_link_libraries(turtlesim_node 
  ${QT_LIBRARIES}
  ${rclcpp_LIBRARIES}
)
ament_target_dependencies(turtlesim_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "Qt4"
)

add_executable(turtle_teleop_key 
  tutorials/teleop_turtle_key.cpp
)

foreach(rmw_implementation ${rmw_implementations2})
  find_package("${rmw_implementation}" REQUIRED)
  get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")
  foreach(typesupport_impl ${typesupport_impls})
    rosidl_target_interfaces(turtle_teleop_key
      ${PROJECT_NAME} ${typesupport_impl}
    )
  endforeach()
endforeach()

target_link_libraries(turtle_teleop_key 
  ${rclcpp_LIBRARIES}
)
ament_target_dependencies(turtle_teleop_key
  "rclcpp"
  "geometry_msgs"
)

install(TARGETS turtlesim_node turtle_teleop_key
    DESTINATION bin)

ament_package()