cmake_minimum_required(VERSION 3.8)
project(motion_tracking_controller)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(legged_rl_controllers REQUIRED)

ament_auto_add_library(
    ${PROJECT_NAME}
    SHARED
    DIRECTORY src
)

# Explicitly bind core ROS dependencies so include paths from legged_rl_controllers
# are always propagated to this target (some environments fail to infer this reliably).
ament_target_dependencies(
    ${PROJECT_NAME}
    controller_interface
    pluginlib
    legged_rl_controllers
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface motion_tracking_controller.xml)

target_include_directories(
    ${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

ament_auto_package(
    INSTALL_TO_SHARE config launch
)
