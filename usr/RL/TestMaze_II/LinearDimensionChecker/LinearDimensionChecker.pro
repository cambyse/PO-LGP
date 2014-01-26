CONFIG += debug
TEMPLATE = app
TARGET = LinearDimensionChecker
QT +=
HEADERS += \
    ../debug.h \
    ../util.h \
    ../util/ColorOutput.h
SOURCES += \
    main.cpp \
    ../util.cpp \
    ../util/ColorOutput.cpp
LIBS += \
    -larmadillo \
    -lgomp
INCLUDEPATH +=
LIBPATH += /usr/share/lib
FORMS +=
RESOURCES +=
QMAKE_CXXFLAGS += \
    -std=c++0x \
    -fopenmp
QMAKE_CC = $$system( if [ `which gcc-4.8 2>/dev/null` ]; then echo "gcc-4.8"; else echo "gcc"; fi )
QMAKE_CXX = $$system( if [ `which g++-4.8 2>/dev/null` ]; then echo "g++-4.8"; else echo "g++"; fi )
message("Project $$TARGET uses:")
message("--> $$QMAKE_CC")
message("--> $$QMAKE_CXX")
