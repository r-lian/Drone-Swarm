add_executable(connect_test
    test_flights/connect_test/connect_test.cpp
)

target_link_libraries(connect_test
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
