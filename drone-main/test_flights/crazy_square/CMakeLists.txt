add_executable(crazy_square
    test_flights/crazy_square/crazy_square.cpp
)

target_link_libraries(crazy_square
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
