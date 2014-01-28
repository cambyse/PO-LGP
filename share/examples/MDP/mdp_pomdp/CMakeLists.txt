get_filename_component(NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(CMAKE_CXX_FLAGS "${MLR_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

# change search order of qmake qt4
find_program(QT_QMAKE_EXECUTABLE NAMES qmake-qt4 qmake4 qmake-qt4 qmake qmake-mac
  PATHS
    ENV QTDIR
    "[HKEY_CURRENT_USER\\Software\\Trolltech\\Versions\\${qt_install_version};InstallDir]"
  PATH_SUFFIXES bin
  DOC "The qmake executable for the Qt installation to use"
)

find_package(Qt4 COMPONENTS QtGui)
if(QT4_FOUND)
  message(WARNING "Disabled use of Qt for ${NAME} because of vtable problem")
#  include(${QT_USE_FILE})
#  include_directories(${QT4_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
#  add_definitions(-DMT_QT ${QT4_DEFINITIONS})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${QT4_EXECUTABLE_COMPILE_FLAGS}")
#  qt4_wrap_ui(${NAME}_FORMS_HEADERS gui.ui)
else(QT4_FOUND)
 message(STATUS "Qt4 not found, building ${NAME} without Gui")
endif(QT4_FOUND)

include(.sources.cmake)
add_executable(x_${NAME} ${SOURCES} ${${NAME}_FORMS_HEADERS})

target_link_libraries(x_${NAME} Core Algo MDP ${QT_LIBRARIES})
add_test(x_${NAME}_test ${NAME} --gtest_output=xml:${NAME}-unit.xml)

