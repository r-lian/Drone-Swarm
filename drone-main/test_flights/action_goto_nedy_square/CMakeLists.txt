add_executable(action_goto_nedy_square
    test_flights/action_goto_nedy_square/action_goto_nedy_square.cpp
)

target_link_libraries(action_goto_nedy_square
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
