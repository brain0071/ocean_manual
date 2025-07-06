cmake_minimum_required(VERSION 3.8)
project(ocean_manual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# naodai
find_package(mavros_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(rclcpp REQUIRED)   

add_executable(run_manual src/run_manual.cpp)
target_include_directories(run_manual PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(run_manual PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# naodai
ament_target_dependencies(run_manual
  rclcpp
  mavros_msgs
  mavlink
)

install(TARGETS run_manual
  DESTINATION lib/${PROJECT_NAME})

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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
