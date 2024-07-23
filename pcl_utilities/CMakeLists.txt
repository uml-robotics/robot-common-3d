cmake_minimum_required(VERSION 3.8)
project(pcl_utilities)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # -Werror
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_utility_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  pcl_utility_msgs
)

# build all files
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

add_executable(voxel_grid_filter_service src/voxel_grid_filter_service.cpp)
ament_target_dependencies(voxel_grid_filter_service ${THIS_PACKAGE_DEPS})

if(BUILD_TESTING)
  # All testing and visualization files are not produced if not in
  # BUILT_TESTING. Launch files will still exist but will fail to launch nodes

  SET(
    THIS_PACKAGE_TEST_DEPS
    rclcpp
    sensor_msgs
    pcl_ros
    pcl_utility_msgs
  )

  add_executable(simple_test_voxel_grid_filter tests/simple_test_voxel_grid_filter.cpp)
  ament_target_dependencies(simple_test_voxel_grid_filter ${THIS_PACKAGE_TEST_DEPS})

  install(TARGETS simple_test_voxel_grid_filter
    EXPORT "export_${PROJECT_NAME}"
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
  )

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

  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS voxel_grid_filter_service
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets("export_${PROJECT_NAME}")
ament_package()
