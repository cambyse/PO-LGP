#include "../testmaze_ii.h"
#include "../Config.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[]) {

    // seed random generator
#ifdef NO_RANDOM
    srand(0);
    srand48(0);
#else
    srand(time(nullptr));
    srand48(time(nullptr));
#endif

    QApplication a(argc, argv);
    TestMaze_II w;
    w.show();
    return a.exec();
}
