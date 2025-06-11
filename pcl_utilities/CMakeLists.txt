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
  add_compile_options(-Wall -Wextra -pedantic -g3) # -Werror
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

# Example code for using sanitizers: addess/thread are mutually exclusive
# Should undefine `__ADDRESS_SANITIZER__` since Eigen changes it's behavior and conflicts
# with PCL code
# add_compile_options(-fsanitize=address,leak,undefined -U__SANITIZE_ADDRESS__)
# add_link_options(-fsanitize=address,leak,undefined)
# add_compile_options(-fsanitize=thread)
# add_link_options(-fsanitize=thread)

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

# ---*---*--- Find all dependency packages ---*---*--
set(THIS_PKG_DEPS
  ament_cmake Eigen3 geometry_msgs
  PCL pcl_conversions pcl_ros
  pcl_utility_msgs rclcpp rcl_interfaces
  sensor_msgs tf2_ros tf2_sensor_msgs)

foreach(PKG IN LISTS THIS_PKG_DEPS)
  find_package(${PKG} REQUIRED)
endforeach()

# ---*--- Handle all standard files ---*---
# These files follow a standard convention
#   that allows them to be conventiently used
set(THIS_PKG_EXPECTED_TARGETS
  concatenate_point_cloud
  euclidean_cluster_extraction
  passthrough_filter
  transform_point_cloud
  voxel_grid_filter)

set(THIS_PKG_EXECS)
set(THIS_PKG_LIB_FILES)

# Stores source files needed for building static libraries and executables
foreach(EXEC IN LISTS THIS_PKG_EXPECTED_TARGETS)
  list(APPEND THIS_PKG_LIB_FILES
    "src/${EXEC}.cpp"
    "include/${PROJECT_NAME}/${EXEC}.hpp")
  add_executable(${EXEC} "src/${EXEC}.cpp")
  list(APPEND THIS_PKG_EXECS ${EXEC})

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/simple_test_${EXEC}.cpp")
    add_executable(simple_test_${EXEC} "src/tests/simple_test_${EXEC}.cpp")
    list(APPEND THIS_PKG_EXECS simple_test_${EXEC})
  endif()
endforeach()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(simple_test_euclidean_cluster_extraction PRIVATE "SHELL:-Wno-pedantic")
endif()

# ---*--- Special cases go here ---*---

# ---*---*--- Create Static/Shared Libraries ---*---*---
# OBJECTS compiles a common set of files that will be turned a static or
# shared lib
add_library(${PROJECT_NAME} ${THIS_PKG_LIB_FILES})

# Recompile the code, exclude the main function from the code
target_compile_definitions(${PROJECT_NAME} PRIVATE -DPCL_UTILITIES_IS_LIBRARY)
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON) # Avoid Exporting Unneeded function from shared lib

# ---*---*--- Handle All Depencencies ---*---*---
foreach(TARGET IN LISTS THIS_PKG_EXECS PROJECT_NAME)
  # Link in required resources
  ament_target_dependencies(${TARGET} ${THIS_PKG_DEPS})

  # Needed for PCL
  target_include_directories(${TARGET} PRIVATE ${PCL_INCLUDE_DIRS})
  target_compile_definitions(${TARGET} PRIVATE ${PCL_DEFINITIONS})
  target_link_directories(${TARGET} PRIVATE ${PCL_LIBRARY_DIRS})

  # Add include directory so files can find this
  target_include_directories(${TARGET}
    PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    PUBLIC  "$<INSTALL_INTERFACE:include>")
endforeach()

# install launch/include directories
install( DIRECTORY include/ DESTINATION include)
install( DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install all executable files
install(
  TARGETS ${THIS_PKG_EXECS}
  DESTINATION lib/${PROJECT_NAME})

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PKG_DEPS})

# ---*---*--- BEGIN TESTING CODE ---*---*---
if(BUILD_TESTING)
  # All testing and visualization files are not produced if not in
  # BUILT_TESTING. Launch files will still exist but will fail to launch nodes

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
