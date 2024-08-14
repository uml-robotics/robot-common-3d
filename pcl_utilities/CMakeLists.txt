cmake_minimum_required(VERSION 3.8)
project(pcl_utilities)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # disable the error message associated with VLAs for now until changed
  add_compile_options(-Wall -Wextra -Wpedantic) # -Werror
endif()

# This is a workaround for a issue within Clang based compilers and BOOST MPL.
# BOOST MPL invokes undefined behavior by casting an integer to an enum
# value that does not exist. PCL uses Boost MPL so many header files do not
# compile. This should be fixed in the next release of Boost, version 1.86.0
#
# Clang failed to diagnose the issue until at least version 16.0.0
# after which it was a error. This detects the issue and opts out of the check
#
# NOTE: Compiler's that use Clang internally but that use different versioning
# always apply the flag do to a lack of documentation. A warning may
#
# All Clang based compilers should match the regex, based on input from:
# https://cmake.org/cmake/help/latest/variable/CMAKE_LANG_COMPILER_ID.html
#
# See for more info: https://github.com/boostorg/mpl/issues/69
if ((CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND
    CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 16.0.0) OR
    CMAKE_CXX_COMPILER_ID MATCHES "(.*Clang|.*LLVM.*)")
  find_package(Boost)
  if (Boost_FOUND AND Boost_VERSION VERSION_LESS_EQUAL 1.86.0)
      add_compile_options(${EXECUTABLE} PUBLIC -Wno-enum-constexpr-conversion)
  endif()
endif()

find_package(PCL REQUIRED)
# PCL may fail to link with certain environments where
# `PCL_ROOT` becomes set to '/'.
# See: https://github.com/PointCloudLibrary/pcl/issues/4661
#
# If PCL is not found:
# - check if PCL is installed
# - check PCL_ROOT is not '/' with: `variable_watch(PCL_ROOT)`
#
# Fix by setting PCL_DIR to the path of the PCL cmake file:
# set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)

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
  PCL
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# build all files
add_library(${PROJECT_NAME} INTERFACE)

add_executable(voxel_grid_filter_service src/voxel_grid_filter_service.cpp)
add_executable(concatenate_point_cloud_service src/concatenate_point_cloud_service.cpp)
ament_target_dependencies(voxel_grid_filter_service ${THIS_PACKAGE_DEPS})
ament_target_dependencies(concatenate_point_cloud_service ${THIS_PACKAGE_DEPS})

set(THIS_PACKAGE_EXECS
  voxel_grid_filter_service
  concatenate_point_cloud_service
)

if(BUILD_TESTING)
  # All testing and visualization files are not produced if not in
  # BUILT_TESTING. Launch files will still exist but will fail to launch nodes

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

  SET(
    THIS_PACKAGE_TEST_DEPS
    rclcpp
    sensor_msgs
    pcl_ros
    pcl_utility_msgs
  )

  add_executable(simple_test_voxel_grid_filter src/tests/simple_test_voxel_grid_filter.cpp)
  add_executable(simple_test_concatenate_point_cloud src/tests/simple_test_concatenate_point_cloud.cpp)

  ament_target_dependencies(simple_test_voxel_grid_filter ${THIS_PACKAGE_TEST_DEPS})
  ament_target_dependencies(simple_test_concatenate_point_cloud ${THIS_PACKAGE_TEST_DEPS})

  list(APPEND THIS_PACKAGE_EXECS
    simple_test_voxel_grid_filter
    simple_test_concatenate_point_cloud
  )

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${THIS_PACKAGE_EXECS}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets("export_${PROJECT_NAME}")
ament_package()
