#include "testmaze_ii.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TestMaze_II w;
    w.show();
    return a.exec();
}
