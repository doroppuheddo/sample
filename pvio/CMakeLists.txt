cmake_minimum_required(VERSION 3.8)
project(pvio)

#Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  # ${colcon_INCLUDE_DIRS}
  # ${CERES_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIR}
  # ${OpenCV_INCLUDE_DIRS}
)

configure_file(cmake/version.h.in include/pvio/version.h)

option(PVIO_DEBUG "..." ON)
option(PVIO_ENABLE_FORENSICS "..." ON)
option(PVIO_ENABLE_THREADING "..." OFF)
option(PVIO_ENABLE_PLANE_CONSTRAINT "..." ON)

# add_executable(main 
#   # src/pvio/config.cpp
#   # src/pvio/core/core.cpp
#   # src/pvio/core/feature_tracker.cpp
#   # src/pvio/core/frontend_worker.cpp
#   # src/pvio/core/initializer.cpp
#   # src/pvio/core/plane_extractor.cpp
#   # src/pvio/core/sliding_window_tracker.cpp
#   # src/pvio/estimation/bundle_adjustor.cpp
#   # src/pvio/estimation/factor.cpp
#   # src/pvio/estimation/pnp.cpp
#   # src/pvio/estimation/preintegrator.cpp
#   # src/pvio/forensics.cpp
#   # src/pvio/geometry/essential.cpp
#   # src/pvio/geometry/homography.cpp
#   # src/pvio/geometry/lie_algebra.cpp
#   # src/pvio/geometry/stereo.cpp
#   # src/pvio/map/frame.cpp
#   # src/pvio/map/map.cpp
#   # src/pvio/map/plane.cpp
#   # src/pvio/map/track.cpp
#   # src/pvio/pvio.cpp
#   # src/pvio/utility/debug.cpp
#   # src/pvio/utility/worker.cpp
#   src/main.cpp
# )

# ament_target_dependencies(main
#   rclcpp
#   std_msgs
#   geometry_msgs

#   Ceres
#   Eigen3
#   OpenCV
#   yaml-cpp
# )

add_executable(feature_tracker_node src/feature_tracker_node.cpp)
target_include_directories(feature_tracker_node PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(feature_tracker_node PRIVATE ${Eigen3_LIBRARIES})
ament_target_dependencies(feature_tracker_node rclcpp std_msgs sensor_msgs Eigen3 cv_bridge image_transport OpenCV)

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
  # main
  feature_tracker_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
