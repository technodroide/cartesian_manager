cmake_minimum_required(VERSION 3.8)
project(cartesian_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(extender_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

generate_parameter_library(
  cartesian_manager_parameters
  src/cartesian_manager.yaml
)

add_library(${PROJECT_NAME}_ros
  src/ros/cartesian_manager.cpp
  src/ros/parameter_parsing.cpp
  src/ros/topic_manager.cpp
)

add_library(${PROJECT_NAME}_core
  src/core/hybrid_orientation_frame.cpp
  src/core/input_manager.cpp
  src/core/manager.cpp
  src/core/shapers/geometric/jaco.cpp
  src/core/shapers/geometric/snake.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_core
  Eigen3
)

target_compile_features(${PROJECT_NAME}_ros PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_ros PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_ros
  Eigen3
  extender_msgs
  generate_parameter_library
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}_core
  cartesian_manager_parameters
)

add_executable(${PROJECT_NAME}_node
  src/ros/main.cpp
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_ros
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY bringup/config bringup/launch docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_ros
    cartesian_manager_parameters
    ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME}_ros)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  Eigen3
  extender_msgs
  generate_parameter_library
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_hybrid_orientation_frame
    test/test_hybrid_orientation_frame.cpp
  )
  target_link_libraries(test_hybrid_orientation_frame
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_cartesian_manager_orientation
    test/test_cartesian_manager_orientation.cpp
  )
  target_link_libraries(test_cartesian_manager_orientation
    ${PROJECT_NAME}_ros
  )
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
