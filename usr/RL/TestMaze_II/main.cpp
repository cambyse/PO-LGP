#include "testmaze_ii.h"
#include "BatchMaze.h"
#include "Data.h"

//#define BATCH_MODE

#include <QtGui>
#include <QApplication>

#include "debug.h"
#include "vector"

int main(int argc, char *argv[])
{

    // seed random generator
    srand(time(nullptr));

    DEBUG_OUT(0, "k_mdp_state_n = " << Data::k_mdp_state_n);
    DEBUG_OUT(0, "state_n = " << Data::state_n);
    DEBUG_OUT(0, "action_n = " << Data::action_n);
    DEBUG_OUT(0, "reward_n = " << Data::reward_n);
    DEBUG_OUT(0, "prediction size = " << Data::k_mdp_state_n*Data::state_n*Data::action_n*Data::reward_n );
    DEBUG_OUT(0, "prediction size = " << Data::k_mdp_state_n*Data::state_n*Data::action_n*Data::reward_n*2 );
    std::vector<short> tmp;
    DEBUG_OUT(0, "max_size = " << tmp.max_size());

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
