cmake_minimum_required(VERSION 2.6)

project(beacons-of-hope)
set(CMAKE_BUILD_TYPE Debug)

add_executable(
    tests
    tests/main.cpp
    tests/positioning_test.cpp
    src/positioning.c
    )

target_link_libraries(
    tests
    CppUTest
    CppUTestExt
    )
