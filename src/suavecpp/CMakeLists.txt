cmake_minimum_required(VERSION 3.8)
project(suavecpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Run this on any executable to ensure correct installation
macro(include_compile_install exec)
  target_include_directories(${exec} PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
  target_compile_features(${exec} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

  message(${exec})
  install(TARGETS ${exec}
          DESTINATION lib/${PROJECT_NAME})
endmacro()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(pcl_conversions REQUIRED)

macro(add_deps_to_exec exec)
  ament_target_dependencies(${exec} MAVSDK)
  ament_target_dependencies(${exec} rclcpp)
  ament_target_dependencies(${exec} nav_msgs)
  ament_target_dependencies(${exec} geometry_msgs)
  ament_target_dependencies(${exec} sensor_msgs)
  ament_target_dependencies(${exec} PCL)
  ament_target_dependencies(${exec} pcl_conversions)
  target_link_libraries(${exec}
          MAVSDK::mavsdk
  )
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  target_link_libraries(${exec} ${PCL_LIBRARIES})
endmacro()

# ADD ALL SOURCE FILES HERE
set(SRC_CODE
  src/nodes/SuaveController.h
  src/nodes/SuaveController.cpp
  src/nodes/SuaveMaskingController.h
  src/nodes/SuaveMaskingController.cpp
)

# suave_masking_controller
add_executable(suave_masking_controller
  src/suave_masking_controller.cpp
  ${SRC_CODE}
)
add_deps_to_exec(suave_masking_controller)
include_compile_install(suave_masking_controller)

# suave_path_planner
add_executable(suave_path_planner
  src/suave_path_planner.cpp
  ${SRC_CODE}
)
add_deps_to_exec(suave_path_planner)
include_compile_install(suave_path_planner)

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
