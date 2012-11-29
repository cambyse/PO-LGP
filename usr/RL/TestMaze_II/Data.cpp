#include "Data.h"

#include <math.h>

#define DEBUG_LEVEL 0
#include "debug.h"

using std::get;
using std::make_tuple;


const char*          Data::action_strings[5] = { " STAY", "   UP", " DOWN", " LEFT", "RIGHT" };
const int            Data::action_n          = NUMBER_OF_ACTIONS;
const int            Data::maze_x_dim        = 2;
const int            Data::maze_y_dim        = 2;
const int            Data::k                 = maze_x_dim+maze_y_dim-2;
const int            Data::state_n           = maze_x_dim*maze_y_dim;
const Data::reward_t Data::min_reward        = 0.0;
const Data::reward_t Data::max_reward        = 1.0;
const Data::reward_t Data::reward_increment  = 1.0;
const int            Data::reward_n          = floor((max_reward-min_reward)/reward_increment)+1;
const int            Data::input_n           = pow(action_n*state_n*reward_n,k+1);
const int            Data::output_n          = state_n*reward_n;

unsigned long Data::idx_from_reward(reward_t reward) {
    if(reward<min_reward) {
        DEBUG_OUT(0,"Error while calculating idx_from_reward. Too low reward." );
        DEBUG_OUT(0,"    reward           = " << reward          );
        DEBUG_OUT(0,"    min_reward       = " << min_reward      );
    }
    if(reward>max_reward) {
        DEBUG_OUT(0,"Error while calculating idx_from_reward. Too high reward." );
        DEBUG_OUT(0,"    reward           = " << reward          );
        DEBUG_OUT(0,"    max_reward       = " << max_reward      );
    }
    double d_reward_idx = (reward-min_reward)/reward_increment;
    if(d_reward_idx != floor(d_reward_idx)) {
        DEBUG_OUT(0,"Error while calculating idx_from_reward. Fractional reward index." );
        DEBUG_OUT(0,"    reward           = " << reward          );
        DEBUG_OUT(0,"    reward_idx       = " << d_reward_idx    );
        DEBUG_OUT(0,"    min_reward       = " << min_reward      );
        DEBUG_OUT(0,"    max_reward       = " << max_reward      );
        DEBUG_OUT(0,"    reward_increment = " << reward_increment);
        DEBUG_OUT(0,"    reward_n         = " << reward_n        );
    }
    return floor(d_reward_idx);
}

Data::reward_t Data::reward_from_idx(unsigned long idx) {
	reward_t reward = min_reward;
	reward += idx*reward_increment;
	if(reward>max_reward) {
		DEBUG_OUT(0, "Error while calculating reward_from_idx. Too high reward:");
		DEBUG_OUT(0,"    reward     = " << reward          );
		DEBUG_OUT(0,"    max_reward = " << max_reward      );
	}
	return reward;
}

unsigned long Data::idx_from_input(input_data_t input_data) {
    unsigned long idx = 0;
    unsigned long block_size = 1;
    int counter = k;
    while(counter>=0) {

        idx += block_size*idx_from_reward(input_data->reward);
        block_size *= reward_n;

        idx += block_size*input_data->state;
        block_size *= state_n;

        idx += block_size*input_data->action;
        block_size *= action_n;

        --input_data;
        --counter;
    }
    return idx;
}

Data::episode_t Data::input_from_idx(unsigned long idx) {
	episode_t episode(k+1);
	episode_iterator_t input_data = episode.begin();
	unsigned long block_size = pow(action_n*state_n*reward_n,k+1);
	while(input_data!=episode.end()) {

		block_size /= action_n;
		input_data->action = idx/block_size;
		idx %= block_size;

		block_size /= state_n;
		input_data->state = idx/block_size;
		idx %= block_size;

		block_size /= reward_n;
		input_data->reward = reward_from_idx(idx/block_size);
		idx %= block_size;

		++input_data;
	}
	if(idx!=0) {
		DEBUG_OUT(0, "Error while calculating input_from_idx");
	}
	return episode;
}

unsigned long Data::idx_from_output(input_data_t input_data) {
    state_t state = input_data->state;
    return state*reward_n + idx_from_reward(input_data->reward);
}

unsigned long Data::idx_from_output(output_data_t output_data) {
    state_t state = output_data.state;
    return state*reward_n + idx_from_reward(output_data.reward);
}

Data::OutputIterator::OutputIterator(): current_state(), current_reward(min_reward) {}

Data::OutputIterator::~OutputIterator() {}

Data::OutputIterator& Data::OutputIterator::operator++() {
    // action stays the same...
    ++current_state;
    if(current_state>=state_n) {
        current_state=state_t();
        current_reward += reward_increment;
    }
    return (*this);
}

Data::output_data_t Data::OutputIterator::operator*() const {
    return output_data_t(current_state,current_reward);
}

bool Data::OutputIterator::end() const {
    return current_reward > max_reward;
}
