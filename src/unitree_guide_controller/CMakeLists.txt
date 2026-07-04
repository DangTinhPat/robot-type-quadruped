cmake_minimum_required(VERSION 3.8)
project(unitree_guide_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CONTROLLER_INCLUDE_DEPENDS
        pluginlib
        rcpputils
        controller_interface
        realtime_tools
        std_msgs
        kdl_parser
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

# Inputs.msg (formerly the separate control_input_msgs package) and the FSM
# base states (formerly the separate controller_common package) both now live
# in this package, since only unitree_guide_controller (and this package's own
# keyboard_input executable) ever consumed them - kept as separate packages
# upstream because their repo has other controllers (ocs2/rl) sharing them,
# which this project doesn't vendor.
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Inputs.msg"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Target is named ${PROJECT_NAME}_lib, not ${PROJECT_NAME}, because
# rosidl_generate_interfaces(${PROJECT_NAME} ...) above already claims the bare
# ${PROJECT_NAME} name as its own custom target - add_library() with the same
# name fails with "another target with the same name already exists". The
# OUTPUT_NAME property below makes it still produce libunitree_guide_controller.so
# so unitree_guide_controller.xml's <library path="unitree_guide_controller">
# doesn't need to change.
add_library(${PROJECT_NAME}_lib SHARED

        src/UnitreeGuideController.cpp

        src/FSM/StatePassive.cpp
        src/FSM/BaseFixedStand.cpp
        src/FSM/StateFixedDown.cpp
        src/FSM/StateFixedStand.cpp
        src/FSM/StateSwingTest.cpp
        src/FSM/StateFreeStand.cpp
        src/FSM/StateBalanceTest.cpp
        src/FSM/StateTrotting.cpp

        src/robot/QuadrupedRobot.cpp
        src/robot/RobotLeg.cpp

        src/control/Estimator.cpp
        src/control/LowPassFilter.cpp
        src/control/BalanceCtrl.cpp

        src/quadProgpp/Array.cc
        src/quadProgpp/QuadProg++.cc

        src/gait/WaveGenerator.cpp
        src/gait/FeetEndCalc.cpp
        src/gait/GaitGenerator.cpp

)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_lib
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)
ament_target_dependencies(
        ${PROJECT_NAME}_lib PUBLIC
        ${CONTROLLER_INCLUDE_DEPENDS}
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(controller_interface unitree_guide_controller.xml)

# Optional teleop node (was the separate keyboard_input package) - only needs
# rclcpp and this package's own Inputs message, none of the controller deps.
add_executable(keyboard_input src/KeyboardInput.cpp)
target_include_directories(keyboard_input
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(keyboard_input PUBLIC rclcpp)
target_link_libraries(keyboard_input PUBLIC "${cpp_typesupport_target}")

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}_lib
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(TARGETS
        keyboard_input
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS} rosidl_default_runtime)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
