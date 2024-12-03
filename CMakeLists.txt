cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(nlohmann_json REQUIRED)

# declare a demo consisting of a single cpp file
function(demo name)
	add_executable(${name} src/${name}.cpp)
	ament_target_dependencies(${name} moveit_task_constructor_core rclcpp)
    target_include_directories(${name} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    target_compile_features(${name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endfunction()

demo(mtc_tutorial)
demo(dual_mtc_tutorial)
demo(dual_mtc_routing)

add_executable(load_scene src/load_scene.cpp)
target_link_libraries(load_scene nlohmann_json::nlohmann_json)
ament_target_dependencies(load_scene
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
)
install(TARGETS load_scene DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# uncomment the line when this package is not in a git repo
#set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()