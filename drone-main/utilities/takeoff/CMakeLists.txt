add_executable(takeoff
    utilities/takeoff/takeoff.cpp
)

target_link_libraries(takeoff
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
