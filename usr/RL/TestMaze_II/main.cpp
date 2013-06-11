#include "testmaze_ii.h"
#include "BatchMaze.h"

#define BATCH_MODE

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
    // seed random generator
    srand(time(nullptr));
    srand48(time(nullptr));

#ifdef BATCH_MODE
    BatchMaze batchmaze;
    return batchmaze.run(argc,argv);
#else
    QApplication a(argc, argv);
    TestMaze_II w;
    w.show();
    return a.exec();

#endif
}
