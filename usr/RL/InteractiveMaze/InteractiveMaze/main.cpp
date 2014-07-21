#include "InteractiveMaze.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    InteractiveMaze w;
    w.show();

    return a.exec();
}
