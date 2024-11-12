cmake_minimum_required(VERSION 3.8)
project(turtlebot3_simulation_intern)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(tf2_ros REQUIRED)
#find_package(turtlebot3_msgs REQUIRED)
#find_package(turtlebot3_description REQUIRED)
#find_package(turtlebot3_simulations REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(turtlebot3_simulation_intern src/turtlebot3_simulation_intern.cpp)

ament_target_dependencies(turtlebot3_simulation_intern rclcpp geometry_msgs OpenCV sensor_msgs cv_bridge std_msgs)

target_link_libraries(turtlebot3_simulation_intern ${OpenCV_LIBS})

install(TARGETS
  turtlebot3_simulation_intern
  DESTINATION lib/${PROJECT_NAME}
)

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
