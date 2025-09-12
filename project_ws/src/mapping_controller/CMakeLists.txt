cmake_minimum_required(VERSION 3.8)

# Set CMP0074 policy to NEW to use <PackageName>_ROOT variables
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(mapping_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bonxai_wrapper REQUIRED)
find_package(PCL REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(
  ${bonxai_wrapper_INCLUDE_DIRS}
)

add_executable(mapping_node src/mapping_node.cpp)

ament_target_dependencies(mapping_node 
  rclcpp 
  std_msgs
  bonxai_wrapper
  PCL
  message_filters
  pcl_conversions
  std_srvs
  tf2_eigen
  tf2_sensor_msgs
)

install(TARGETS mapping_node
  DESTINATION lib/${PROJECT_NAME})

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
