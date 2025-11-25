cmake_minimum_required(VERSION 3.8)
project(image_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)


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

add_executable(active_ir_processor src/active_ir_processor.cpp)
add_executable(passive_ir_processor src/passive_ir_processor.cpp)
add_executable(depth_to_rgb src/depth_to_rgb.cpp)
add_executable(occlusion src/occlusion.cpp)

ament_target_dependencies(active_ir_processor
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
ament_target_dependencies(passive_ir_processor
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)
ament_target_dependencies(occlusion
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)
ament_target_dependencies(depth_to_rgb
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  Eigen3
)
target_include_directories(depth_to_rgb PUBLIC ${EIGEN3_INCLUDE_DIRS})


install(TARGETS
  active_ir_processor
  passive_ir_processor
  occlusion
  depth_to_rgb
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
