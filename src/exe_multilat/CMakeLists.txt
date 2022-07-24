cmake_minimum_required (VERSION 3.19.0)

find_package(multilat)

add_executable(exe_multilat main.cpp)
target_link_libraries(exe_multilat PRIVATE multilat)
