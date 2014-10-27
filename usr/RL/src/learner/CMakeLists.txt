find_package(Qt5Widgets)
find_package(Qt5Svg)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(learner
    AdjacencyOperator.h
    ConjunctiveAdjacency.cpp
    ConjunctiveAdjacency.h
    FeatureLearner.cpp
    FeatureLearner.h
    TemporallyExtendedFeatureLearner.cpp
    TemporallyExtendedFeatureLearner.h
    TemporallyExtendedLinearQ.cpp
    TemporallyExtendedLinearQ.h
    TemporallyExtendedModel.cpp
    TemporallyExtendedModel.h
    UTree.cpp
    UTree.h
)

target_include_directories(learner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(learner PUBLIC Qt5::Widgets Qt5::Svg)
