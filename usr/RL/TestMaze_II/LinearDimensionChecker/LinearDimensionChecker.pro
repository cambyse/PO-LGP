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
