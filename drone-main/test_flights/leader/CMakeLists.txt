add_executable(leader
    test_flights/leader/leader.cpp
)

target_link_libraries(leader
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
