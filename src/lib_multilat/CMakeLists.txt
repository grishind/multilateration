cmake_minimum_required (VERSION 3.19.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(OpenMP REQUIRED)
add_library (multilat INTERFACE
        basics.h
        scenegenerator.h
        solver.h
        solvers/locate_bancroft.h
        solvers/locate_beck.h
        solvers/locate_grad.h
        solvers/locate_linear.h
        solvers/locate_simple.h
        solvers/locate_zhil.h
        statistics.h
        utils.h
        vector3d.h
        )

target_include_directories(multilat INTERFACE . ~/Eigen/eigen-3.4.0/)

target_link_libraries(multilat INTERFACE OpenMP::OpenMP_CXX)
