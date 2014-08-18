#ifndef RANDOMELEMENTS_H_
#define RANDOMELEMENTS_H_

#include "../Representation/AbstractAction.h"
#include "../Representation/AbstractObservation.h"
#include "../Representation/AbstractReward.h"

AbstractAction::ptr_t get_random_action(bool reuse = false);

AbstractAction::ptr_t get_random_minimal_action();

AbstractAction::ptr_t get_random_maze_action();

AbstractAction::ptr_t get_random_augmented_maze_action();

AbstractAction::ptr_t get_random_cheese_maze_action();

AbstractAction::ptr_t get_random_button_action();

AbstractObservation::ptr_t get_random_observation(bool reuse = false);

AbstractObservation::ptr_t get_random_minimal_observation();

AbstractObservation::ptr_t get_random_maze_observation();

AbstractObservation::ptr_t get_random_cheese_maze_observation();

AbstractObservation::ptr_t get_random_button_observation();

AbstractReward::ptr_t get_random_reward(bool reuse = false);

AbstractReward::ptr_t get_random_minimal_reward();

AbstractReward::ptr_t get_random_listed_reward();

#endif /* RANDOMELEMENTS_H_ */
