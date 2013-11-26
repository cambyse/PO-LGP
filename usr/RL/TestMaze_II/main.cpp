#include "testmaze_ii.h"
#include "BatchMaze.h"
#include "Config.h"

#include <QtGui>
#include <QApplication>

//#define FORCE_BATCH

int main(int argc, char *argv[]) {

    // seed random generator
#ifdef NO_RANDOM
    srand(0);
    srand48(0);
#else
    srand(time(nullptr));
    srand48(time(nullptr));
#endif

#if defined(BATCH_MODE) || defined(BATCH_MODE_QUIET) || defined(FORCE_BATCH)
    BatchMaze batchmaze;
    return batchmaze.run(argc,argv);
#else
    QApplication a(argc, argv);
    TestMaze_II w;
    w.show();
    return a.exec();

#endif
}
