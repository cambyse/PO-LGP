#include <gtest/gtest.h>

#include "../util/QtUtil.h"
#include "../util/ColorOutput.h"

#include "../Maze/Maze.h"
#include "../Planning/LookAheadPolicy.h"

#define DEBUG_LEVEL 2
#include "../debug.h"

using std::vector;
using std::shared_ptr;

TEST(PlannerTest, LookAheadSearch) {

    // use standard typedefs
    typedef AbstractAction::ptr_t      action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t      reward_ptr_t;
    typedef Instance                   instance_t;
    typedef double                     probability_t;

    // do both prune and don't prune
    for(bool prune_search_tree : {false,true}) {

        if(prune_search_tree) {
            DEBUG_OUT(1,ColorOutput::bold() << "Pruning" << ColorOutput::reset_all() << " search tree...");
        } else {
            DEBUG_OUT(1,ColorOutput::bold() << "Not pruning" << ColorOutput::reset_all() << " search tree...");
        }

        // initialize environment and planner
        shared_ptr<Maze> maze(new Maze());
        LookAheadPolicy planner(0.5,maze,prune_search_tree,10000);

        const instance_t * maze_instance = maze->get_current_instance();
        instance_t * current_instance = instance_t::create(
            maze_instance->action,
            maze_instance->observation,
            maze_instance->reward
            );

        // do several planned steps
        for(int step_idx=0; step_idx<10; ++step_idx) {

            action_ptr_t action = planner.get_action(current_instance);

            // actually perform transition and print results
            observation_ptr_t observation_to;
            reward_ptr_t reward;
            maze->perform_transition(action,observation_to,reward);
            current_instance = current_instance->append_instance(action, observation_to, reward);
            switch(step_idx%5) {
            case 0:{
                EXPECT_EQ(MazeAction("left"),action);
                EXPECT_EQ(MazeObservation(2,2,0,0),observation_to);
                EXPECT_EQ(ListedReward({0,1},0),reward);
                break;
            }
            case 1:
                EXPECT_EQ(MazeAction("right"),action);
                EXPECT_EQ(MazeObservation(2,2,1,0),observation_to);
                EXPECT_EQ(ListedReward({0,1},1),reward);
                break;
            case 2:
                EXPECT_EQ(MazeAction("down"),action);
                EXPECT_EQ(MazeObservation(2,2,1,1),observation_to);
                EXPECT_EQ(ListedReward({0,1},1),reward);
                break;
            case 3:
                EXPECT_EQ(MazeAction("left"),action);
                EXPECT_EQ(MazeObservation(2,2,0,1),observation_to);
                EXPECT_EQ(ListedReward({0,1},0),reward);
                break;
            case 4:
                EXPECT_EQ(MazeAction("up"),action);
                EXPECT_EQ(MazeObservation(2,2,0,0),observation_to);
                EXPECT_EQ(ListedReward({0,1},0),reward);
                break;
            default:
                DEBUG_DEAD_LINE;
                EXPECT_TRUE(false);
            }

            // print nice pictures
            if(DEBUG_LEVEL>1) {
                maze->print_transition(action, observation_to, reward);
            }

        }

        // delete instance
        delete current_instance;

    }
}
