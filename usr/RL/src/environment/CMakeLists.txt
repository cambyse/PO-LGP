find_package(Qt5Core)
find_package(Qt5Widgets)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(environment
    Environment.h
    HistoryObserver.cpp
    HistoryObserver.h
    PredictiveEnvironment.cpp
    PredictiveEnvironment.h
    Predictor.cpp
    Predictor.h
    SpaceManager.cpp
    SpaceManager.h
    Visualizer.cpp
    Visualizer.h
)

target_include_directories(environment PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(environment PUBLIC Qt5::Core Qt5::Widgets)
