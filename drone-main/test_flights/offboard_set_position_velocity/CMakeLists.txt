add_executable(offboard_set_position_velocity
    test_flights/offboard_set_position_velocity/offboard_set_position_velocity.cpp
)

target_link_libraries(offboard_set_position_velocity
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
