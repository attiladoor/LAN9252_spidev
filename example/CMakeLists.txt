cmake_minimum_required(VERSION 3.2.0)
project(lan9225-spidev)

set(EXAMPLE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/EasyCAT_Test.cpp)

add_executable(ethercat-example ${EXAMPLE_SOURCES})

target_link_libraries(ethercat-example lib-lan9225-spidev)
