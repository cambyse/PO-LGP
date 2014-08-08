#ifndef RANDOMELEMENTS_H_
#define RANDOMELEMENTS_H_

#include "../Representation/AbstractAction.h"
#include "../Representation/AbstractObservation.h"
#include "../Representation/AbstractReward.h"

AbstractAction::ptr_t get_random_action();

AbstractAction::ptr_t get_random_minimal_action();

AbstractAction::ptr_t get_random_maze_action();

AbstractAction::ptr_t get_random_augmented_maze_action();

AbstractAction::ptr_t get_random_cheese_maze_action();

AbstractAction::ptr_t get_random_button_action();

AbstractObservation::ptr_t get_random_observation();

AbstractObservation::ptr_t get_random_minimal_observation();

AbstractObservation::ptr_t get_random_maze_observation();

AbstractObservation::ptr_t get_random_cheese_maze_observation();

AbstractReward::ptr_t get_random_reward();

AbstractReward::ptr_t get_random_minimal_reward();

AbstractReward::ptr_t get_random_listed_reward();

#endif /* RANDOMELEMENTS_H_ */
