add_executable(offboard_goto_nedy_square
    test_flights/offboard_goto_nedy_square/offboard_goto_nedy_square.cpp
)

target_link_libraries(offboard_goto_nedy_square
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
