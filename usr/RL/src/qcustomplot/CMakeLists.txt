find_package(Qt5Widgets)
find_package(Qt5PrintSupport)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(qcustomplot
    qcustomplot.cpp
    qcustomplot.h
)

target_include_directories(qcustomplot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(qcustomplot PUBLIC Qt5::Widgets Qt5::PrintSupport)
