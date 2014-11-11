file(GLOB CANDIDATE_SOURCES *.cpp *.c src/*.cpp)
string(REGEX REPLACE main\\.[a-zA-Z0-9_.-]+.cpp "" SOURCES "${CANDIDATE_SOURCES}")
#message(STATUS ${CMAKE_CURRENT_SOURCE_DIR} ${SOURCES})
