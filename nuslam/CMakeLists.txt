cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nuturtle_control REQUIRED)
find_package(nuturtle_interfaces REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(${ARMADELLO_INCLUDE_DIRS})

set(SLAM_DEPENDENCIES
    rclcpp
    tf2_ros
    rcl_interfaces
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    nuturtle_control
    nuturtle_interfaces
)

set(ODOMETRY_DEPENDENCIES
    rclcpp
    tf2_ros
    rcl_interfaces
    sensor_msgs
    nav_msgs
    geometry_msgs
    nuturtle_control
    nuturtle_interfaces
)

set(LANDMARKS_DEPENDENCIES
    rclcpp
    rcl_interfaces
    sensor_msgs
    visualization_msgs
)

add_executable(slam src/slam.cpp)
ament_target_dependencies(slam
    ${SLAM_DEPENDENCIES}
)
target_link_libraries(slam
    turtlelib::turtlelib
    ${ARMADELLO_LIBRARIES}
)

add_executable(odometry src/odometry.cpp)
ament_target_dependencies(odometry
    ${ODOMETRY_DEPENDENCIES}
)
target_link_libraries(odometry
    turtlelib::turtlelib
    ${ARMADELLO_LIBRARIES}
)

add_executable(landmarks src/landmarks.cpp)
ament_target_dependencies(landmarks
    ${LANDMARKS_DEPENDENCIES}
)
target_link_libraries(landmarks
    turtlelib::turtlelib
    ${ARMADELLO_LIBRARIES}
)

install(TARGETS
    slam
    odometry
    landmarks
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

if(NOT CMAKE_CROSSCOMPILING)
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
endif()

ament_package()
