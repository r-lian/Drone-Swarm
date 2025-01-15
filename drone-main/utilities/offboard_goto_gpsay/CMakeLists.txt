add_executable(offboard_goto_gpsay
    utilities/offboard_goto_gpsay/offboard_goto_gpsay.cpp
)

target_link_libraries(offboard_goto_gpsay
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
