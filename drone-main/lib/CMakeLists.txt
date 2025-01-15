cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)

add_library(helpers helpers.cpp mutex/mutex.cpp thread_tracker/thread_tracker.cpp command_line_argument_parser/command_line_argument_parser.cpp drone/drone.cpp)
# add_executable(helpers helpers.cpp mutex/mutex.cpp timer/timer.cpp command_line_arguments_parser/command_line_arguments_parser.cpp drone/drone.cpp)
list(APPEND EXTRA_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(helpers PUBLIC
    ${EXTRA_INCLUDES}
)
target_link_libraries(helpers PUBLIC
    MAVSDK::mavsdk
    MAVSDK::mavsdk_action
    MAVSDK::mavsdk_offboard
    MAVSDK::mavsdk_telemetry
)
