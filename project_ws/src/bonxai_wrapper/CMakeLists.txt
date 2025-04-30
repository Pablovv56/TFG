cmake_minimum_required(VERSION 3.8)

# Set CMP0074 policy to NEW to use <PackageName>_ROOT variables
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(bonxai_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  Bonxai/bonxai_ros/include
  Bonxai/bonxai_map/include
  onxai/bonxai_core/include
)

add_library(bonxai_server
  Bonxai/bonxai_ros/src/bonxai_server.cpp
)

# Add Bonxai map library
add_library(bonxai_map
  Bonxai/bonxai_map/src/probabilistic_map.cpp
  Bonxai/bonxai_map/src/pcl_utils.cpp
)

target_link_libraries(bonxai_server bonxai_map Eigen3::Eigen)
target_link_libraries(bonxai_map ${PCL_LIBRARIES})

target_include_directories(bonxai_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Bonxai/bonxai_map/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Bonxai/bonxai_core/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

# Install the library
install(TARGETS bonxai_server bonxai_map
  EXPORT bonxai_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the export set for the bonxai_server targ
install(
  EXPORT bonxai_server
  FILE bonxai_serverConfig.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Add the Bonxai include directory to the install path
install(DIRECTORY Bonxai/bonxai_ros/include/ DESTINATION include)
install(DIRECTORY Bonxai/bonxai_map/include/ DESTINATION include)
install(DIRECTORY Bonxai/bonxai_core/include/ DESTINATION include)

# Export the include path for downstream packages
ament_export_include_directories(include)

# Add dependencies
ament_target_dependencies(bonxai_server
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  tf2_ros
  pcl_conversions
  PCL
  tf2_eigen
)

# Export the library
ament_export_targets(bonxai_server HAS_LIBRARY_TARGET)
ament_export_libraries(bonxai_server bonxai_map)
ament_export_dependencies(rclcpp sensor_msgs std_srvs tf2_ros pcl_conversions PCL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
