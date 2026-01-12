cmake_minimum_required(VERSION 3.10)
project(unilidar_driver)

set(SERIAL_INSTALL OFF CACHE BOOL "Disable serial library installation" FORCE)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG")

include_directories(include)

link_directories(lib/${CMAKE_SYSTEM_PROCESSOR})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

# Library configuration
set(unilidar_driver_SRCS
    src/udp_handler.cpp
    src/unitree_lidar_packet_parser.cpp
    src/unitree_lidar_reader_imp.cpp
)
add_library(${PROJECT_NAME} STATIC
    ${unilidar_driver_SRCS}
)

if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC _GNU_SOURCE)
endif()

# Include directories
target_include_directories(${PROJECT_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/3rdparty/serial/include>
)

# Link libraries
if(WIN32)
    target_link_libraries(${PROJECT_NAME} ws2_32)
endif()

# Add serial
add_subdirectory(3rdparty/serial)
if(TARGET serial)
    target_compile_options(serial PRIVATE
        -Wno-sign-compare
        -Wno-unused-but-set-variable
    )
    target_link_libraries(${PROJECT_NAME} serial)
endif()

# Example program
add_executable(example_lidar_udp
  examples/example_lidar_udp.cpp
)
target_link_libraries(example_lidar_udp ${PROJECT_NAME})

add_executable(example_lidar_serial
    examples/example_lidar_serial.cpp
)
target_link_libraries(example_lidar_serial ${PROJECT_NAME})

add_executable(set_ip_address
    examples/set_ip_address.cpp
)
target_link_libraries(set_ip_address ${PROJECT_NAME})

add_executable(set_to_serial_mode
    examples/set_to_serial_mode.cpp
)
target_link_libraries(set_to_serial_mode ${PROJECT_NAME})

add_executable(set_to_udp_mode
    examples/set_to_udp_mode.cpp
)
target_link_libraries(set_to_udp_mode ${PROJECT_NAME})

# Install targets if needed (optional)
install(TARGETS unilidar_driver
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(TARGETS example_lidar_udp
    RUNTIME DESTINATION bin
)

install(TARGETS example_lidar_serial
    RUNTIME DESTINATION bin
)

install(TARGETS set_ip_address
    RUNTIME DESTINATION bin
)

install(TARGETS set_to_serial_mode
    RUNTIME DESTINATION bin
)

install(TARGETS set_to_udp_mode
    RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install serial headers (if needed by your public API)
install(DIRECTORY 3rdparty/serial/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Generate and install CMake config files
include(CMakePackageConfigHelpers)

# Generate the config file that includes the exports
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/unilidar_driverConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/unilidar_driverConfig.cmake
    INSTALL_DESTINATION lib/cmake/unilidar_driver
)

# Generate the version file
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/unilidar_driverConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

# Install the config and version files
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/unilidar_driverConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/unilidar_driverConfigVersion.cmake
    DESTINATION lib/cmake/unilidar_driver
)

# Export targets
install(EXPORT unilidar_driverTargets
    FILE unilidar_driverTargets.cmake
    NAMESPACE unilidar_driver::
    DESTINATION lib/cmake/unilidar_driver
)

# Modify the library install to include export
install(TARGETS unilidar_driver serial
    EXPORT unilidar_driverTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)