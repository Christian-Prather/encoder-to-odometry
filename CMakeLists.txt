cmake_minimum_required(VERSION 3.8)
project(encoder_to_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(luci_messages REQUIRED)

add_executable(converter 
    src/main.cpp
    src/odometry.cpp
    src/encoder_converter.cpp
)



target_include_directories(converter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${luci_messages_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

target_link_libraries(converter PUBLIC 
  rclcpp::rclcpp 
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${luci_messages_LIBRARIES}
  ${tf2_ros_LIBRARIES}
 )

 install (TARGETS converter DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
