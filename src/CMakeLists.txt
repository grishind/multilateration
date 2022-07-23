cmake_minimum_required (VERSION 3.19.0)
project (multilateration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


add_subdirectory(lib_multilat)
add_subdirectory(exe_multilat)
