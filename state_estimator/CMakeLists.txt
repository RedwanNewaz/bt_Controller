
add_executable(create3_state_estimator state_estimator_node.cpp)

ament_target_dependencies(
        create3_state_estimator
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
)

target_link_libraries(create3_state_estimator
        ${rclcpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${irobot_create_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${visualization_msgs_LIBRARIES}
        yaml-cpp
        COM

        )