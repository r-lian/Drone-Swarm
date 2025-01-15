add_executable(partner
    test_flights/partner/partner.cpp
)

target_link_libraries(partner
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
