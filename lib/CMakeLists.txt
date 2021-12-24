cmake_minimum_required(VERSION 3.2.0)
project(lan9225-spidev)

file(GLOB SOURCES_DIR
	${CMAKE_CURRENT_SOURCE_DIR}/*
)

add_library(lib-lan9225-spidev ${CMAKE_CURRENT_SOURCE_DIR}/EasyCAT.cpp)
target_include_directories(lib-lan9225-spidev PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

