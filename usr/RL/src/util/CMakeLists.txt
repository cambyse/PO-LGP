cmake_minimum_required(VERSION 2.8)

find_package(Qt5Core)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(util
    ChiSquareTest.cpp
    ChiSquareTest.h
    ColorOutput.cpp
    ColorOutput.h
    Commander.cpp
    Commander.h
    debug_exclude.h
    debug.h
    function_signature.h
    graph_plotting.h
    KolmogorovSmirnovDist.cpp
    KolmogorovSmirnovDist.h
    KolmogorovSmirnovTest.cpp
    KolmogorovSmirnovTest.h
    lbfgs_codes.cpp
    lbfgs_codes.h
    macro_lib.h
    pretty_printer.h
    ProgressBar.cpp
    ProgressBar.h
    QtUtil.cpp
    QtUtil.h
    return_tuple.h
    return_tuple_macros.h
    template_lib.h
    util.cpp
    util.h
    softmax.h
)

target_include_directories(util PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(util PUBLIC Qt5::Core)
