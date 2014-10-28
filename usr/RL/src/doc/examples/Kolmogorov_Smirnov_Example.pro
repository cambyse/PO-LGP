CONFIG += debug
TARGET = Kolmogorov_Smirnov_Example
QT += core \
    gui \
    svg
HEADERS += \
    ../util/KolmogorovSmirnovTest.h
SOURCES += \
    Kolmogorov_Smirnov_Example.cpp \
    ../util/KolmogorovSmirnovTest.cpp
INCLUDEPATH +=
LIBS +=
FORMS +=
RESOURCES +=
QMAKE_CXXFLAGS += -std=c++0x

