add_executable(offboard_set_position_square
    test_flights/offboard_set_position_square/offboard_set_position_square.cpp
)

target_link_libraries(offboard_set_position_square
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
