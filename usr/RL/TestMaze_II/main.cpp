#include "testmaze_ii.h"
#include "BatchMaze.h"

#define BATCH_MODE

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
#ifdef BATCH_MODE
    BatchMaze batchmaze;
    batchmaze.run();
    return 0;
#else
    QApplication a(argc, argv);
    TestMaze_II w;
    w.show();
    return a.exec();

#endif
}
