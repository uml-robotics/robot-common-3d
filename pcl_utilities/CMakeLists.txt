cmake_minimum_required(VERSION 3.8)
project(pcl_utilities)

#[[ This code should not be compiled with `-Ofast` or `-ffastmath`
for the following reasons:
 - These utilities rely on IEEE 754 Compliance.
 - PCL uses floating point operations internally, and enabling these
   flags can result in unexpected behavior in client code.

It is the users responsibility to add the appropriate flags on
non Clang/GNU-based compilers (including. non-posix frontend variants.)
]]
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---*---*--- Handle GNU and Clang Based Compilers ---*---*---
# Handle GNU and Clang compilers
# GNU Based Compilers: GNU, Intel
# CLANG Based Compilers: Clang, AppleClang, IBMClang, ...
#   - CAVEAT: `clang-cl` will not work and needs to be excluded
#
# See full listing: `https://cmake.org/cmake/help/latest/variable/CMAKE_LANG_COMPILER_ID.html`
set(IS_COMPILER_CXX_GNU false)
set(IS_COMPILER_CXX_CLANG false)

if(
  CMAKE_CXX_COMPILER_ID MATCHES ".*(Clang|LLVM).*" AND
  NOT CMAKE_CXX_COMPILER_FRONTEND_VARIANT EQUAL "MSVC"
)
  set(IS_COMPILER_CXX_CLANG true)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(IS_COMPILER_CXX_GNU true)
endif()

# ---*---*--- Stricter Compilation Flags ---*---*---
if(IS_COMPILER_CXX_GNU OR IS_COMPILER_CXX_CLANG)
  # -Werror flag is removed due to errors involving redundent syntax,
  # anonymous structs/unions in pcl/point_types.h
  add_compile_options(-Wall -Wextra -pedantic) # -Werror
endif()

#[[ ---*---*--- Boost Error Workaround ---*---*---
This is a workaround for a issue that existed within Clang based
compilers and BOOST MPL. BOOST MPL invoked undefined behavior by
casting an integer to an enum value that does not exist. PCL uses
Boost MPL so many header files do not compile. This bug was patched
for Boost 1.86.0.

See for more info: https://github.com/boostorg/mpl/issues/69

In Clang Versions >= 20, this error cannot be supressed. You have 3 options:
  -  At the time of reading, you shoud check for an apt package
    with a correct version.
  - Source install a correct version of boost, beware that linking against files
    with another version of boost may trigger ODR violations at link time.
  - Patch the header manually: https://github.com/boostorg/mpl/pull/77/files/8499ae7e4ff0cf798367ebe6ea9fb991aa43db6c
#diff-49c45a857bb9eaf779f64be204dc1ca6cfdf3801775428c03acfabf8104fe25d
  for what changes need to be made.
]]
if(IS_COMPILER_CXX_CLANG)
  find_package(Boost)
  if(Boost_FOUND AND Boost_VERSION VERSION_LESS_EQUAL 1.86.0)
    add_compile_options(-Wno-enum-constexpr-conversion)
  endif()
endif()

#[[ ---*---*--- Add Sanitizers ---*---*---
In order to use these sanitizers for testing, apply the following steps:
  - These are for DEBUGGING only, never use for hardening a binary (except for
    safe-stack, cfi, and scudo)
  - Asan and Tsan cannot be used together
  - If using Asan, also add `-U__ADDRESS_SANITIZER__`, Eigen changes its
    allocation scheme if Asan is in use, which creates conflicts with PCL
    library code leading to a SIGSEGV.
  - If you see errors about unmapped memory regions or repeated messages
    [DEADLY SIGNAL] from Asan. The number of bits for Asan are
    above what is handleable for sanitizers
      - Launch with `ros2 launch --launch-prefix='setarch --addr-no-randomize'`
      - Run With `ros2 run --prefix='setarch --addr-no-randomize'`
  - Asan raises an error with a known issue in ROS2 has a known error with deleting
    pointers with the wrong type, this is irrelevent for linux using the default allocator.
    You can disable it with `export ASAN_OPTIONS=new_delete_type_mismatch=0`
  - Tsan expects all dependencies to be compiled with Tsan, we don't do that
    You can ignore all mutex and signal errors that are not from the current
    executable. In order to reduce error messages, only leave race conditions
    detected: `export TSAN_OPTIONS=report_signal_unsafe=0:report_mutex_bugs=0:report_destroy_locked=0:detect_deadlocks=0`
  - Nsan only works with Clang++ version >= 20, useful for idenfitying accumationg
    floating point precision issues in code You need to use `-DPCL_NO_PRECOMPILE`
    to detect issues in PCL code. These detections are not bugs but may require
    more attention is percision is of high imporantance, though these warnings will
    infrequently be problematic.
For more information go to:
    Tutorial / Nsan: https://www.intel.com/content/www/us/en/developer/articles/technical/
find-bugs-quickly-using-sanitizers-with-oneapi-compiler.html
    Asan: https://github.com/google/sanitizers/wiki/AddressSanitizer
          https://clang.llvm.org/docs/AddressSanitizer.html
    Lsan: https://github.com/google/sanitizers/wiki/AddressSanitizerLeakSanitizer
    Ubsan: https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html
    Tsan: https://github.com/google/sanitizers/wiki/threadsanitizercppmanual

The below shows adding sanitizers per project source, specify `CXXFLAGS` for global changes.
]]
# if (IS_COMPILER_CXX_GNU OR IS_COMPILER_CXX_CLANG)
#   message("Building \"${PROJECT_NAME}\" with sanitizers")

#   add_compile_options(-g3)

#   # add_compile_options(-fsanitize=address,leak,undefined -U__SANITIZE_ADDRESS__)
#   # add_link_options(-fsanitize=address,leak,undefined)
#   # <OR>
#   # add_compile_options(-fsanitize=thread)
#   # add_link_options(-fsanitize=thread)
#   # <OR> (>=clang++-20)
#   # add_definitions(-DPCL_NO_PRECOMPILE)
#   # add_compile_options(-fsanitize=numerical)
#   # add_link_options(-fsanitize=numerical)
# endif()

#[[PCL may fail to link with certain environments where
`PCL_ROOT` becomes set to '/'.
See: https://github.com/PointCloudLibrary/pcl/issues/4661

If PCL is not found:
- check if PCL is installed
- check PCL_ROOT is not '/' with: `variable_watch(PCL_ROOT)`
]]
# Fix by setting PCL_DIR to the path of the PCL cmake file:
# set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)

# ---*---*--- Find all dependency packages ---*---*---
set(THIS_PKG_DEPS
  ament_cmake Eigen3 geometry_msgs
  PCL pcl_conversions pcl_ros
  pcl_utility_msgs rclcpp rcl_interfaces
  sensor_msgs tf2_ros tf2_sensor_msgs)

foreach(PKG IN LISTS THIS_PKG_DEPS)
  find_package(${PKG} REQUIRED)
endforeach()

# ---*--- Handle All Standard Files ---*---
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
  list(APPEND THIS_PKG_LIB_FILES "src/${EXEC}.cpp")
  add_executable(${EXEC} "src/${EXEC}.cpp")
  list(APPEND THIS_PKG_EXECS ${EXEC})

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/${EXEC}.hpp")
    list(APPEND THIS_PKG_LIB_FILES "include/${PROJECT_NAME}/${EXEC}.hpp")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/simple_test_${EXEC}.cpp")
    add_executable(simple_test_${EXEC} "src/tests/simple_test_${EXEC}.cpp")
    list(APPEND THIS_PKG_EXECS simple_test_${EXEC})
  endif()
endforeach()

# ---*--- Special cases go here ---*---

# A PCL header causes warnings in gcc/clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(simple_test_euclidean_cluster_extraction PRIVATE "SHELL:-Wno-pedantic")
endif()

# ---*---*--- Create Static/Shared Libraries ---*---*---
# OBJECTS compiles a common set of files that will be turned a static or
# shared lib
add_library(${PROJECT_NAME} ${THIS_PKG_LIB_FILES})

# Recompile the code, exclude the main function from the code
# `-PCL_UTILITIES_IS_LIBRARY` will conditionally exclude the main driver function
target_compile_definitions(${PROJECT_NAME} PRIVATE -DPCL_UTILITIES_IS_LIBRARY)
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON) # Avoid exporting unneeded function from shared lib

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

# ---*---*--- Install All Resouces Into Directories ---*---*---

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

# ---*---*--- Linting and Testing ---*---*---
# Unit tests should be added in here
if(BUILD_TESTING)
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
