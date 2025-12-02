cmake_minimum_required(VERSION 3.8)
project(capture)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# capture executable
add_executable(capture src/capture.cpp)
ament_target_dependencies(capture 
  cv_bridge
  image_transport
  OpenCV
  rclcpp
)

# viewer executable
add_executable(viewer src/viewer.cpp)
ament_target_dependencies(viewer 
  cv_bridge
  image_transport
  OpenCV
  rclcpp
)

# Install executables
install( 
  TARGETS capture viewer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# # Install include directories
# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
