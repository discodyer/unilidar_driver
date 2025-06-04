cmake_minimum_required(VERSION 3.0)
project(unilidar_driver)

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
)
add_library(${PROJECT_NAME}
    ${unilidar_driver_SRCS}
)
# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Example program
add_executable(example_lidar_udp
  examples/example_lidar_udp.cpp
)
target_link_libraries(example_lidar_udp ${PROJECT_NAME} )

# add_executable(example_lidar_serial
#     examples/example_lidar_serial.cpp
# )
# target_link_libraries(example_lidar_serial  libunilidar_sdk2.a  )

# add_executable(set_ip_address
#     examples/set_ip_address.cpp
# )
# target_link_libraries(set_ip_address  libunilidar_sdk2.a )

# add_executable(set_to_serial_mode
#     examples/set_to_serial_mode.cpp
# )
# target_link_libraries(set_to_serial_mode  libunilidar_sdk2.a )

# add_executable(set_to_udp_mode
#     examples/set_to_udp_mode.cpp
# )
# target_link_libraries(set_to_udp_mode  libunilidar_sdk2.a )

# Install targets if needed (optional)
install(TARGETS unilidar_driver
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(TARGETS example_lidar_udp
    RUNTIME DESTINATION bin
)