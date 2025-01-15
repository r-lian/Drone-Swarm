add_executable(action_goto_gpsay
    utilities/action_goto_gpsay/action_goto_gpsay.cpp
)

target_link_libraries(action_goto_gpsay
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
