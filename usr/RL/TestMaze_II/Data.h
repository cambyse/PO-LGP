/*
 * Data.h
 *
 *  Created on: Nov 9, 2012
 *      Author: robert
 */

#ifndef DATA_H_
#define DATA_H_

#include <vector>
#include <tuple>

namespace Data {

// general
typedef int                                   action_t;
typedef int                                   state_t;
typedef double                                reward_t;
typedef double                                probability_t;
typedef std::tuple<action_t,state_t,reward_t> data_point_t;
typedef std::vector<data_point_t>             episode_t;
typedef episode_t::const_iterator             const_episode_iterator_t;

// features
typedef const_episode_iterator_t input_data_t;
typedef const data_point_t       output_data_t;

} // end namespace

#endif /* DATA_H_ */
