cmake_minimum_required(VERSION 3.8)
project(pcl_utilities)

# This code should not be compiled with `-Ofast` or `-ffastmath`
# for the following reasons:
#  - These utilities rely on IEEE 754 Compliance.
#  - PCL uses floating point operations internally, and enabling these
#    flags can result in unexpected behavior in client code.
#
# It is the users responsibility to add the appropriate flags on
# non Clang/GNU-based compilers (including. non-posix frontend variants.)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # disable the error message associated with VLAs for now until changed
  # -Werror flag is removed due to errors involving redundent syntax,
  # anonymous structs/unions in pcl/point_types.h
  add_compile_options(-Wall -Wextra -pedantic) # -Werror
endif()

# This is a workaround for a issue that existed within Clang based
# compilers and BOOST MPL. BOOST MPL invoked undefined behavior by
# casting an integer to an enum value that does not exist. PCL uses
# Boost MPL so many header files do not compile. This bug was patched
# in Boost 1.86.0.
#
# Clang failed to diagnose the issue until at least version 16.0.0
# after which it was a error. This detects the issue and opts out of the check
#
# NOTE: Compiler's that use LLVM/Clang internally but that use a different
# versioning schemes always have the flag applied.
#
# All Clang based compilers should match the regex, based on input from:
# https://cmake.org/cmake/help/latest/variable/CMAKE_LANG_COMPILER_ID.html
#
# See for more info: https://github.com/boostorg/mpl/issues/69
if((CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND
    CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 16.0.0) OR
    CMAKE_CXX_COMPILER_ID MATCHES "(.+Clang|.*LLVM.*)")
  find_package(Boost)
  if(Boost_FOUND AND Boost_VERSION VERSION_LESS_EQUAL 1.86.0)
    add_compile_options(-Wno-enum-constexpr-conversion)
  endif()
endif()

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
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_utility_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THIS_PACKAGE_DEPS
  PCL
  pcl_ros
  pcl_conversions
  pcl_utility_msgs
  rclcpp
  rcl_interfaces
  sensor_msgs
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
# PCL_NO_PRECOMPILE is required to avoid address sanitizer issues in clang,
# see https://github.com/PointCloudLibrary/pcl/issues/5831
add_definitions(${PCL_DEFINITIONS} -DPCL_NO_PRECOMPILE)

# build all files
add_library(${PROJECT_NAME} INTERFACE)

add_executable(concatenate_point_cloud_service src/concatenate_point_cloud_service.cpp)
add_executable(euclidean_cluster_extraction_service src/euclidean_cluster_extraction_service.cpp)
add_executable(passthrough_filter_service src/passthrough_filter_service.cpp)
add_executable(voxel_grid_filter_service src/voxel_grid_filter_service.cpp)

ament_target_dependencies(concatenate_point_cloud_service ${THIS_PACKAGE_DEPS})
ament_target_dependencies(euclidean_cluster_extraction_service ${THIS_PACKAGE_DEPS})
ament_target_dependencies(passthrough_filter_service ${THIS_PACKAGE_DEPS})
ament_target_dependencies(voxel_grid_filter_service ${THIS_PACKAGE_DEPS})

set(THIS_PACKAGE_EXECS
  concatenate_point_cloud_service
  euclidean_cluster_extraction_service
  passthrough_filter_service
  voxel_grid_filter_service
)

if(BUILD_TESTING)
  # All testing and visualization files are not produced if not in
  # BUILT_TESTING. Launch files will still exist but will fail to launch nodes

  set(THIS_PACKAGE_TEST_DEPS
    Eigen3
    PCL
    pcl_conversions
    pcl_ros
    pcl_utility_msgs
    rclcpp
    rcl_interfaces
    sensor_msgs
  )

  add_executable(simple_test_concatenate_point_cloud src/tests/simple_test_concatenate_point_cloud.cpp)
  add_executable(simple_test_euclidean_cluster_extraction src/tests/simple_test_euclidean_cluster_extraction.cpp)
  add_executable(simple_test_passthrough_filter src/tests/simple_test_passthrough_filter.cpp)
  add_executable(simple_test_voxel_grid_filter src/tests/simple_test_voxel_grid_filter.cpp)

  ament_target_dependencies(simple_test_concatenate_point_cloud ${THIS_PACKAGE_TEST_DEPS})
  ament_target_dependencies(simple_test_euclidean_cluster_extraction ${THIS_PACKAGE_TEST_DEPS})
  ament_target_dependencies(simple_test_passthrough_filter ${THIS_PACKAGE_TEST_DEPS})
  ament_target_dependencies(simple_test_voxel_grid_filter ${THIS_PACKAGE_TEST_DEPS})

  list(APPEND THIS_PACKAGE_EXECS
    simple_test_concatenate_point_cloud
    simple_test_passthrough_filter
    simple_test_euclidean_cluster_extraction
    simple_test_voxel_grid_filter
  )

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # these options will make debugging memory issues and undefined behavior easier
    # -g3 -O0 : make debugging easier using GDB
    # -D _FORTIFY_SOURCE=2 : adds memory corruption protections
    add_definitions(-D_FORTIFY_SOURCE=2)
    add_compile_options(-g3 -O0)

    # Need to remove `-Wpedantic` since PCL uses constructs that are not
    # Standard compliant which produces warnings under `pedantic`
    # Use `SHELL:` prefix to avoid deduplication, if `-Wno-pedantic` is added
    # elsewhere.
    target_compile_options(simple_test_euclidean_cluster_extraction
      PRIVATE "SHELL:-Wno-pedantic")
  endif()

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
