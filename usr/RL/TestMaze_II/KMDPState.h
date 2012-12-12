/*
 * KMDPState.h
 *
 *  Created on: Dec 12, 2012
 *      Author: robert
 */

#ifndef KMDPSTATE_H_
#define KMDPSTATE_H_

#include <deque>
#include "Data.h"

class KMDPState {

public:

    KMDPState();

    virtual ~KMDPState();

    void new_state(const Data::data_point_t&);
    void new_state(const Data::action_t&, const Data::state_t&, const Data::reward_t&);

    Data::k_mdp_state_t get_k_mdp_state();

protected:

    std::deque<Data::data_point_t> k_mdp_state_deque;

};

#endif /* KMDPSTATE_H_ */







