# Configures Cube
add_executable(cube test_flights/cube/cube.cpp)
target_link_libraries(cube
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
    MAVSDK::mavsdk
    ${EXTRA_LIBS}
)
target_include_directories(cube PUBLIC
	"${PROJECT_BINARY_DIR}"
	${EXTRA_INCLUDES}
)
