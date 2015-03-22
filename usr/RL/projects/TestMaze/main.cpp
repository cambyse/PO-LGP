#include "testmaze.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TestMaze w;
    w.show();
    return a.exec();
}
