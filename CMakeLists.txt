cmake_minimum_required(VERSION 3.5)
project(usv_nav)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# find_package(cv_bridge REQUIRED)
# find_package(target_bbox_msgs REQUIRED)
# find_package(OpenCV 3 REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

include_directories(include 
  ${PROJECT_SOURCE_DIR}/include
 )
  
file(GLOB_RECURSE SOURCES_DIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(${PROJECT_NAME} ${SOURCES_DIRS})
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs sensor_msgs)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(FILES launch/usv_nav_launch.py launch/usv_nav_launch.py
  DESTINATION share/${PROJECT_NAME}/launch)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
