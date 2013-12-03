#include <gtest/gtest.h>

#include "../util.h"
#include "../AbstractAction.h"
#include "../Maze/MazeAction.h"
#include "../Maze/AugmentedMazeAction.h"

#define DEBUG_LEVEL 0
#include "../debug.h"

namespace {

    TEST(AbstractIteratableSpace, AbstractAction) {

        std::vector<std::shared_ptr<AbstractAction> > action_vector;
        action_vector.push_back(std::make_shared<MazeAction>(MazeAction::ACTION::DOWN));
        action_vector.push_back(std::make_shared<AugmentedMazeAction>(AugmentedMazeAction::ACTION::DOWN,AugmentedMazeAction::TAG::TAG_2));

        int action_type_idx = 0;
        // for all action types (represented by one specific action of each type)
        for(auto action_type : action_vector) {
            DEBUG_OUT(1,"This action: " << action_type->print());
            DEBUG_OUT(1,"    Action space:");
            // go through all actions of the corresponding action space
            int match_counter = 0;
            int match_idx = -1;
            for(auto action_in_space : *action_type) {
                DEBUG_OUT(1,"    " << action_in_space->print() );
                // make sure only a single action matches (the one we use for
                // iterating its action space)
                int action_idx = 0;
                for(auto action : action_vector) {
                    if(*action_in_space==*action) {
                        ++match_counter;
                        match_idx = action_idx;
                        DEBUG_OUT(1,"        X " << action->print());
                    } else {
                        DEBUG_OUT(1,"        - " << action->print());
                    }
                    ++action_idx;
                }
            }
            EXPECT_EQ(1,match_counter) << "expecting exactly one action from each action type/space";
            EXPECT_EQ(action_type_idx,match_idx) << "match should be the action currently used to iterate its space";
            ++action_type_idx;
        }
    }

}; // end namespace
