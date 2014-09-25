cmake_minimum_required(VERSION 2.8)
project(beacon-visualizer)

include_directories(../dependencies/ ../)

add_library(beacon SHARED 
    ../kalman.c 
    ../positioning.c
    ../dependencies/platform-abstraction/mock/mutex.c
)
