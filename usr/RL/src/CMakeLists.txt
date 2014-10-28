cmake_minimum_required(VERSION 3.0)

# add current directory to include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# build type: release / debug / unit_tests
if(CMAKE_BUILD_TYPE MATCHES unit_tests|UNIT_TESTS)
    # only GTest executable, not MazeGUI and BatchWorker executables
    message(STATUS "Build Type: UNIT_TESTS")
    add_definitions(-DUNIT_TESTS -O3)
    add_subdirectory(GTest)
elseif(CMAKE_BUILD_TYPE MATCHES release|Release|RELEASE)
    # only MazeGUI and BatchWorker executables, not GTest executable
    message(STATUS "Build Type: RELEASE")
    add_definitions(-DRELEASE -DNDEBUG -O3)
    add_subdirectory(gui)
    add_subdirectory(batchworker)
elseif(CMAKE_BUILD_TYPE MATCHES debug|Debug|DEBUG)
    # only MazeGUI and BatchWorker executables, not GTest executable
    message(STATUS "Build Type: DEBUG")
    add_definitions(-DDEBUG -O0)
    add_subdirectory(gui)
    add_subdirectory(batchworker)
else()
    # expect explicit definition of build type
    message(FATAL_ERROR "Unkown or unspecified Build Type. Use '-DCMAKE_BUILD_TYPE=<type>' with <type> being RELEASE, DEBUG, or UNIT_TESTS when calling cmake")
endif()

# add all other directories
add_subdirectory(ButtonWorld)
add_subdirectory(CheeseMaze)
add_subdirectory(environment)
add_subdirectory(learner)
add_subdirectory(Maze)
add_subdirectory(optimization)
add_subdirectory(planning)
add_subdirectory(representation)
add_subdirectory(util)

# print defines
get_directory_property(DirDefs DIRECTORY ${CMAKE_CURRENT_LIST_DIR} COMPILE_DEFINITIONS)
foreach(d ${DirDefs})
    message(STATUS "Using Define: ${d}")
endforeach()
