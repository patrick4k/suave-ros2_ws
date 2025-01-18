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

# ADD ALL SOURCE FILES HERE
add_executable(suave_controller
  src/suave_controller.cpp
  src/nodes/SuaveController.h
  src/nodes/SuaveController.cpp
  src/nodes/SuaveMaskingController.h
  src/nodes/SuaveMaskingController.cpp
)

ament_target_dependencies(suave_controller MAVSDK)
ament_target_dependencies(suave_controller rclcpp)
ament_target_dependencies(suave_controller nav_msgs)
ament_target_dependencies(suave_controller geometry_msgs)
ament_target_dependencies(suave_controller sensor_msgs)
ament_target_dependencies(suave_controller PCL)
ament_target_dependencies(suave_controller pcl_conversions)
target_link_libraries(suave_controller
        MAVSDK::mavsdk
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(suave_controller ${PCL_LIBRARIES})

include_compile_install(suave_controller)

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
