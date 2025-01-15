add_executable(telemetry utilities/telemetry/telemetry.cpp)
target_link_libraries(telemetry PUBLIC
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
target_include_directories(telemetry PUBLIC
	"${PROJECT_BINARY_DIR}"
	${EXTRA_INCLUDES}
)
