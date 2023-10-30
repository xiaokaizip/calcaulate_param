cmake_minimum_required(VERSION 3.26)

project(c)
LINK_LIBRARIES(m)
include_directories(
        Inc
)

file(GLOB_RECURSE SOURCES "src/*.*")

add_executable(loop_control ${SOURCES})


