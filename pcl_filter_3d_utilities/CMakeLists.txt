cmake_minimum_required(VERSION 3.8)
project(pcl_filter_3d_utilities)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # -Werror
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_filter_3d_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# build all files
set(PCL_DEPENDENCIES
  rclcpp sensor_msgs pcl_conversions pcl_filter_3d_msgs pcl_ros
)

set(COLLIDER_DEPS
  rclcpp rcl_interfaces moveit_ros_planning_interface moveit_msgs geometry_msgs pcl_filter_3d_msgs
)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(pcl_voxel_grid_filter_service src/pcl_voxel_grid_filter_service.cpp)

ament_target_dependencies(pcl_voxel_grid_filter_service ${PCL_DEPENDENCIES})


install(TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # these options will make debugging memory issues and undefined behavior easier
    # -g3 -O0 : make debugging easier using GDB
    # -fstack-protector-all : will crash if stack corruption is detected
    # -D FORTIFY_SOURCE=2 : adds memory corruption protections
    # -ftrapv : detects signed overflow which is undefined per the C++ standard
    # -Wl,-z,relro : a linker flag to move immutable memory into ROM which
    #     will help detect invalid writes
    add_compile_options(-g3 -O0 -fstack-protector-all -D FORTIFY_SOURCE=2 -ftrapv -Wl,-z,relro)
  endif()

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  # file(GLOB AMENT_LINT_AUTO_FILE_EXCLUDE_
  #   build/* install/* log/*
  # )

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets("export_${PROJECT_NAME}")
ament_package()
