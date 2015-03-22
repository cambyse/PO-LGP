#include "RandomElements.h"

#include <util/util.h>

#include "MinimalEnvironmentExample/MinimalAction.h"
#include <Maze/MazeAction.h>
#include <CheeseMaze/CheeseMazeAction.h>
#include <ButtonWorld/ButtonAction.h>
#include <Maze/AugmentedMazeAction.h>
#include "MinimalEnvironmentExample/MinimalObservation.h"
#include <Maze/MazeObservation.h>
#include <CheeseMaze/CheeseMazeObservation.h>
#include <representation/UniqueObservation.h>
#include "MinimalEnvironmentExample/MinimalReward.h"
#include <representation/ListedReward.h>

#include <vector>

#include <util/debug.h>

using util::random_select;
using std::vector;

AbstractAction::ptr_t get_random_action() {
    AbstractAction::ptr_t action;
    switch(random_select<AbstractAction::ACTION_TYPE>({
                AbstractAction::ACTION_TYPE::MINIMAL,
                    AbstractAction::ACTION_TYPE::MAZE_ACTION,
                    AbstractAction::ACTION_TYPE::AUGMENTED_MAZE_ACTION,
                    AbstractAction::ACTION_TYPE::CHEESE_MAZE_ACTION,
                    AbstractAction::ACTION_TYPE::BUTTON_ACTION
                    })) {
    case AbstractAction::ACTION_TYPE::MINIMAL:
        action = get_random_minimal_action();
        break;
    case AbstractAction::ACTION_TYPE::MAZE_ACTION:
        action = get_random_maze_action();
        break;
    case AbstractAction::ACTION_TYPE::AUGMENTED_MAZE_ACTION:
        action = get_random_augmented_maze_action();
        break;
    case AbstractAction::ACTION_TYPE::CHEESE_MAZE_ACTION:
        action = get_random_cheese_maze_action();
        break;
    case AbstractAction::ACTION_TYPE::BUTTON_ACTION:
        action = get_random_button_action();
        break;
    default:
        DEBUG_ERROR("Unexpected type");
        action = AbstractAction::ptr_t();
    }
    return action;
}

AbstractAction::ptr_t get_random_minimal_action() {
    return AbstractAction::ptr_t(new MinimalAction(random_select<MinimalAction::ACTION>({
                        MinimalAction::ACTION::STAY,
                        MinimalAction::ACTION::CHANGE
                        })));
}

AbstractAction::ptr_t get_random_maze_action() {
    return AbstractAction::ptr_t(new MazeAction(random_select<MazeAction::ACTION>({
                        MazeAction::ACTION::UP,
                        MazeAction::ACTION::DOWN,
                        MazeAction::ACTION::LEFT,
                        MazeAction::ACTION::RIGHT,
                        MazeAction::ACTION::STAY
                        })));
}

AbstractAction::ptr_t get_random_augmented_maze_action() {
    return AbstractAction::ptr_t(new AugmentedMazeAction(random_select<AugmentedMazeAction::ACTION>({
                        AugmentedMazeAction::ACTION::UP,
                        AugmentedMazeAction::ACTION::DOWN,
                        AugmentedMazeAction::ACTION::LEFT,
                        AugmentedMazeAction::ACTION::RIGHT,
                        AugmentedMazeAction::ACTION::STAY
                        }),
            random_select<AugmentedMazeAction::TAG>({
                        AugmentedMazeAction::TAG::TAG_0,
                        AugmentedMazeAction::TAG::TAG_1,
                        AugmentedMazeAction::TAG::TAG_2
                        })));
}

AbstractAction::ptr_t get_random_cheese_maze_action() {
    return AbstractAction::ptr_t(new CheeseMazeAction(random_select<CheeseMazeAction::ACTION>({
                        CheeseMazeAction::ACTION::NORTH,
                        CheeseMazeAction::ACTION::SOUTH,
                        CheeseMazeAction::ACTION::EAST,
                        CheeseMazeAction::ACTION::WEST
                        })));
}

AbstractAction::ptr_t get_random_button_action() {
    int array_length = rand()%5 + 1;
    vector<bool> action_array(array_length);
    for(int idx : util::Range(array_length)) {
        action_array[idx] = rand()%2==0;
    }
    return AbstractAction::ptr_t(new ButtonAction(array_length, action_array));
}

AbstractObservation::ptr_t get_random_observation() {
    AbstractObservation::ptr_t observation;
    switch(random_select<AbstractObservation::OBSERVATION_TYPE>({
                AbstractObservation::OBSERVATION_TYPE::MINIMAL,
                    AbstractObservation::OBSERVATION_TYPE::MAZE_OBSERVATION,
                    AbstractObservation::OBSERVATION_TYPE::CHEESE_MAZE_OBSERVATION,
                    AbstractObservation::OBSERVATION_TYPE::UNIQUE_OBSERVATION
                    })) {
    case AbstractObservation::OBSERVATION_TYPE::MINIMAL:
        observation = get_random_minimal_observation();
        break;
    case AbstractObservation::OBSERVATION_TYPE::MAZE_OBSERVATION:
        observation = get_random_maze_observation();
        break;
    case AbstractObservation::OBSERVATION_TYPE::CHEESE_MAZE_OBSERVATION:
        observation = get_random_cheese_maze_observation();
        break;
    case AbstractObservation::OBSERVATION_TYPE::UNIQUE_OBSERVATION:
        observation = get_unique_observation();
        break;
    default:
        DEBUG_ERROR("Unexpected type");
        observation = AbstractObservation::ptr_t();
    }
    return observation;
}

AbstractObservation::ptr_t get_random_minimal_observation() {
    return AbstractObservation::ptr_t(new MinimalObservation(random_select<MinimalObservation::OBSERVATION>({
                        MinimalObservation::OBSERVATION::RED,
                        MinimalObservation::OBSERVATION::GREEN
                        })));
}

AbstractObservation::ptr_t get_random_maze_observation() {
    int x_dim = rand()%5 + 1;
    int y_dim = rand()%5 + 1;
    int x_pos = rand()%x_dim;
    int y_pos = rand()%y_dim;
    return AbstractObservation::ptr_t(new MazeObservation(x_dim,y_dim,x_pos,y_pos));
}

AbstractObservation::ptr_t get_random_cheese_maze_observation() {
    return AbstractObservation::ptr_t(new CheeseMazeObservation(random_select<CheeseMazeObservation::OBSERVATION>({
                        CheeseMazeObservation::OBSERVATION::N,
                        CheeseMazeObservation::OBSERVATION::NW,
                        CheeseMazeObservation::OBSERVATION::NS,
                        CheeseMazeObservation::OBSERVATION::NE,
                        CheeseMazeObservation::OBSERVATION::EW,
                        CheeseMazeObservation::OBSERVATION::ESW
                        })));
}

AbstractObservation::ptr_t get_unique_observation() {
    return AbstractObservation::ptr_t(new UniqueObservation());
}

AbstractReward::ptr_t get_random_reward() {
    AbstractReward::ptr_t reward;
    switch(random_select<AbstractReward::REWARD_TYPE>({
                    AbstractReward::REWARD_TYPE::MINIMAL,
                    AbstractReward::REWARD_TYPE::LISTED_REWARD
                    })) {
    case AbstractReward::REWARD_TYPE::MINIMAL:
        reward = get_random_minimal_reward();
        break;
    case AbstractReward::REWARD_TYPE::LISTED_REWARD:
        reward = get_random_listed_reward();
        break;
    default:
        DEBUG_ERROR("Unexpected type");
        reward = AbstractReward::ptr_t();
    }
    return reward;
}

AbstractReward::ptr_t get_random_minimal_reward() {
    return AbstractReward::ptr_t(new MinimalReward(random_select<MinimalReward::REWARD>({
                        MinimalReward::REWARD::NO_REWARD,
                        MinimalReward::REWARD::SOME_REWARD
                        })));
}

AbstractReward::ptr_t get_random_listed_reward() {
    int list_length = rand()%5 + 1;
    vector<AbstractReward::value_t> reward_list(list_length);
    for(unsigned int idx=0; idx<reward_list.size(); ++idx) {
        reward_list[idx] = idx;
    }
    return AbstractReward::ptr_t(new ListedReward(reward_list,rand()%list_length));
}
