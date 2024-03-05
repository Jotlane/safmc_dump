cmake_minimum_required(VERSION 3.8)
project(aruco_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR /usr/local/lib/cmake/opencv)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


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


add_executable(detect_markers src/detect_markers.cpp)
add_executable(calibrate_camera_charuco src/calibrate_camera_charuco.cpp)

ament_target_dependencies(detect_markers rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV tf2 tf2_ros geometry_msgs tf2_geometry_msgs)
ament_target_dependencies(calibrate_camera_charuco rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)


install(TARGETS
	detect_markers
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	calibrate_camera_charuco
	DESTINATION lib/${PROJECT_NAME}
)
ament_package()
