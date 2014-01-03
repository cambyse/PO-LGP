#ifndef RANDOMELEMENTS_H_
#define RANDOMELEMENTS_H_

#include "../AbstractAction.h"
#include "../AbstractObservation.h"
#include "../AbstractReward.h"

AbstractAction::ptr_t get_random_action();

AbstractAction::ptr_t get_random_minimal_action();

AbstractAction::ptr_t get_random_maze_action();

AbstractAction::ptr_t get_random_augmented_maze_action();

AbstractObservation::ptr_t get_random_observation();

AbstractObservation::ptr_t get_random_minimal_observation();

AbstractObservation::ptr_t get_random_maze_observation();

AbstractReward::ptr_t get_random_reward();

AbstractReward::ptr_t get_random_minimal_reward();

AbstractReward::ptr_t get_random_listed_reward();

#endif /* RANDOMELEMENTS_H_ */
