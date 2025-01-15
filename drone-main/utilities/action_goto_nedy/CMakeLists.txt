add_executable(action_goto_nedy
    utilities/action_goto_nedy/action_goto_nedy.cpp
)

target_link_libraries(action_goto_nedy
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
