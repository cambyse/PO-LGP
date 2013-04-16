#include "testmaze_ii.h"
#include "BatchMaze.h"
#include "Data.h"

//#define BATCH_MODE

#include <QtGui>
#include <QApplication>

#include "debug.h"
#include "vector"

#include "util.h"
#include "Representation/Action.h"
#include "Representation/Reward.h"
#include "Representation/State.h"

int main(int argc, char *argv[])
{

    using util::INVALID;
    Action action(Action::LEFT);

    std::cout << "Actions:" << std::endl;
    for(ActionIt aIt(Action::END_ACTION-1); aIt!=INVALID; --aIt) {
        std::cout <<
            aIt << ": " <<
            aIt.action_string() <<
            (aIt==action ? " is equal to " : " is not equal to ") <<
            action.action_string() <<
            std::endl;
    }
    std::cout << "Rewards:" << std::endl;
    for(RewardIt rIt(Reward::max_reward); rIt!=INVALID; --rIt) {
        std::cout << rIt << std::endl;
    }
    std::cout << "States:" << std::endl;
    for(StateIt sIt(State::max_state); sIt!=INVALID; --sIt) {
        std::cout << sIt << std::endl;
    }

    return 0;

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
