#include "testmaze_ii.h"
#include "BatchMaze.h"
#include "Config.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
    // seed random generator
    srand(time(nullptr));
    srand48(time(nullptr));

#if defined(BATCH_MODE) || defined(BATCH_MODE_QUIET)
    BatchMaze batchmaze;
    return batchmaze.run(argc,argv);
#else
    QApplication a(argc, argv);
    TestMaze_II w;
    w.show();
    return a.exec();

#endif
}
