add_executable(offboard_goto_nedy
    utilities/offboard_goto_nedy/offboard_goto_nedy.cpp
)

target_link_libraries(offboard_goto_nedy
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
