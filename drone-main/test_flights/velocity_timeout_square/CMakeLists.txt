add_executable(velocity_timeout_square
    test_flights/velocity_timeout_square/square_wait_for_offboard_mode.cpp
)

target_link_libraries(velocity_timeout_square
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
