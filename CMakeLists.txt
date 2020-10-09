cmake_minimum_required(VERSION 3.5)
project(microros_moveit2_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(microros_moveit2_demo src/microros_moveit2_demo.cpp)
ament_target_dependencies(microros_moveit2_demo
  moveit_ros_planning_interface
  Boost
)

install(TARGETS microros_moveit2_demo
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
