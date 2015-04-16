TEMPLATE = lib
CONFIG += staticlib \
          debug
TARGET = util
QT += core \
      gui \
      svg \
      printsupport
HEADERS += ./*.h
SOURCES += ./*.cpp
LIBS += -llbfgs \
    -lemon \
    -larmadillo \
    -lgomp \
    -lgtest \
    -lgsl \
    -lgslcblas
LIBPATH += /usr/share/lib
QMAKE_CXXFLAGS += -std=c++0x \
                  -fopenmp
