cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

project(drone VERSION 1.0 LANGUAGES CXX)

find_package(MAVSDK REQUIRED)

# Global Helper Functions
# add_subdirectory(lib)

add_subdirectory(lib) # Uses CMakelists in lib.

list(APPEND EXTRA_LIBS helpers)
list(APPEND EXTRA_INCLUDES "${PROJECT_SOURCE_DIR}/lib")

# Utilities CMake Files.
file(GLOB utilities_projects "utilities/*/CMakeLists.txt")
message(STATUS "utilities cmake files")
foreach (utility ${utilities_projects})
	message(STATUS "${utility}")
	include("${utility}")
endforeach()

# Test Flights CMake Files.
file(GLOB test_flights_projects "test_flights/*/CMakeLists.txt")
message(STATUS "test_flights cmake files")
foreach (test_flight_project ${test_flights_projects})
	message(STATUS "${test_flight_project}")
	include("${test_flight_project}")
endforeach()
