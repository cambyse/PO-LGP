#include <gtest/gtest.h>

#include "../Maze/Maze.h"
#include "../KMarkovCRF.h"

#define DEBUG_LEVEL 1
#include "../debug.h"

using std::vector;
using std::shared_ptr;

TEST(LearnerTest, LookAheadSearch) {

    // use standard typedefs
    typedef AbstractAction::ptr_t      action_ptr_t;
    typedef AbstractObservation::ptr_t observation_ptr_t;
    typedef AbstractReward::ptr_t      reward_ptr_t;
    typedef Instance                   instance_t;
    typedef double                     probability_t;

        // initialize environment and learner
        Maze maze;
        KMarkovCRF learner();

        // get/set spaces and features
        learner.set_spaces(maze);
        learner.set_features(maze);

        const instance_t * maze_instance = maze.get_current_instance();
        instance_t * current_instance = instance_t::create(
            maze_instance->action,
            maze_instance->observation,
            maze_instance->reward
            );

        // do several planned steps
        for(int step_idx=0; step_idx<10; ++step_idx) {

            // debugging
            if(DEBUG_LEVEL>2) {
                DEBUG_OUT(0,"Before planning:");
                learner.print_tree_statistics();
            }

            // do planning and select action
            action_ptr_t action;
            int max_tree_size = 10000;
            if(prune_search_tree && step_idx>0) {
                learner.fully_expand_tree<Maze>(
                    maze,
                    max_tree_size
                    );
            } else {
                learner.clear_tree();
                learner.build_tree<Maze>(
                    current_instance,
                    maze,
                    max_tree_size
                    );
            }
            action = learner.get_optimal_action();

            // actually the perform transition and print results
            observation_ptr_t observation_to;
            reward_ptr_t reward;
            maze.perform_transition(action,observation_to,reward);
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
                // get action string
                const char * as;
                if(action==MazeAction("up")) {
                    as = "↑";
                } else if(action==MazeAction("down")) {
                    as = "↓";
                } else if(action==MazeAction("right")) {
                    as = "→";
                } else if(action==MazeAction("left")) {
                    as = "←";
                } else if(action==MazeAction("stay")) {
                    as = "●";
                } else {
                    EXPECT_TRUE(false) << "unexpected action (" << action << ")";
                }
                // get reward colorization
                QString rc, rs = ColorOutput::reset_all().c_str();
                if(reward->get_value()==0) {
                    rc = "";
                } else if(reward->get_value()==1) {
                    rc = ColorOutput::fg_red().c_str();
                } else {
                    EXPECT_TRUE(false) << "unexpected reward " << reward;
                }
                // print the thing
                if(observation_to==MazeObservation(2,2,0,0)) {
                    std::cout << "┏━━━┳━━━┓" << std::endl;
                    std::cout << "┃◗"<<rc<<"●"<<rs<<" ╲   ┃" << std::endl;
                    std::cout << "┣━━━╋━━━┫"<<as << std::endl;
                    std::cout << "┃   ┃   ┃" << std::endl;
                    std::cout << "┗━━━┻━━━┛" << std::endl;
                } else if(observation_to==MazeObservation(2,2,0,1)) {
                    std::cout << "┏━━━┳━━━┓" << std::endl;
                    std::cout << "┃◗  ╲   ┃" << std::endl;
                    std::cout << "┣━━━╋━━━┫"<<as << std::endl;
                    std::cout << "┃ "<<rc<<"●"<<rs<<" ┃   ┃" << std::endl;
                    std::cout << "┗━━━┻━━━┛" << std::endl;
                } else if(observation_to==MazeObservation(2,2,1,0)) {
                    std::cout << "┏━━━┳━━━┓" << std::endl;
                    std::cout << "┃◗  ╲ "<<rc<<"●"<<rs<<" ┃" << std::endl;
                    std::cout << "┣━━━╋━━━┫"<<as << std::endl;
                    std::cout << "┃   ┃   ┃" << std::endl;
                    std::cout << "┗━━━┻━━━┛" << std::endl;
                } else if(observation_to==MazeObservation(2,2,1,1)) {
                    std::cout << "┏━━━┳━━━┓" << std::endl;
                    std::cout << "┃◗  ╲   ┃" << std::endl;
                    std::cout << "┣━━━╋━━━┫"<<as << std::endl;
                    std::cout << "┃   ┃ "<<rc<<"●"<<rs<<" ┃" << std::endl;
                    std::cout << "┗━━━┻━━━┛" << std::endl;
                } else {
                    EXPECT_TRUE(false) << "unexpected observation (" << observation_to << ")";
                }
            }

            // debugging
            if(DEBUG_LEVEL>2) {
                DEBUG_OUT(0,"After planning, before pruning:");
                learner.print_tree_statistics();
            }

            // sanity check
            probability_t prob = learner.get_predicted_transition_probability<Maze>(action, observation_to, reward, maze);
            if(prob==0) {
                probability_t prob_maze = maze.get_prediction(current_instance->const_it()-1, action, observation_to, reward);
                DEBUG_ERROR("Warning: Transition with predicted probability of zero for (" << action << "," << observation_to << "," << reward << ") (Maze predicts " << prob_maze << ")" );
                EXPECT_TRUE(false) << "see output above";
            }

            // prune tree
            if(prune_search_tree) {
                learner.prune_tree(action,current_instance,maze);
            }

            if(DEBUG_LEVEL>2) {
                DEBUG_OUT(0,"After pruning:");
                learner.print_tree_statistics();
            }
        }
    }
}
