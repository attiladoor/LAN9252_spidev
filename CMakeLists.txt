cmake_minimum_required(VERSION 3.2.0)
project(lan9225-spidev)


set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(${REPO_ROOT}/lib)
add_subdirectory(${REPO_ROOT}/example)
