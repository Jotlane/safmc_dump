cmake_minimum_required(VERSION 3.8)
project(safmc_2024)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR /use/local/lib/cmake/opencv)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(safmc_msgs REQUIRED)
# offboard_control.cpp
add_executable(offboard_controller src/offboard_controller.cpp)
ament_target_dependencies(offboard_controller rclcpp px4_msgs geometry_msgs tf2 tf2_ros geometry_msgs safmc_msgs)
install(TARGETS offboard_controller DESTINATION lib/${PROJECT_NAME})

# activate_offboard.cpp
add_executable(activate_offboard src/activate_offboard.cpp)
ament_target_dependencies(activate_offboard rclcpp px4_msgs)
install(TARGETS activate_offboard DESTINATION lib/${PROJECT_NAME})


add_executable(image_converter src/aruco/image_converter.cpp)
add_executable(calibration src/aruco/calibrate_camera_charuco.cpp)

ament_target_dependencies(image_converter rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV safmc_msgs)
ament_target_dependencies(calibration rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV safmc_msgs)

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

install(TARGETS
	image_converter
	DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
	calibration
	DESTINATION lib/${PROJECT_NAME}
)install(TARGETS
        )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
