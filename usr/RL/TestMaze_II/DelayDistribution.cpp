#include "DelayDistribution.h"

#include <map>

#define DEBUG_STRING "DelayDist: "
#define DEBUG_LEVEL 1
#include "debug.h"

using util::INVALID;

using std::vector;
using std::map;
using std::tuple;
using std::make_tuple;
using std::get;

DelayDistribution::probability_t DelayDistribution::get_fixed_delay_probability(
    const observation_ptr_t& s1,
    const observation_ptr_t& s2,
    const idx_t& delay
    ) {
    // determine unnormalized delay probability
    probability_t prob = 0;
    size_t normalization = 0;
    if(number_of_data_points>0) {
        for(instance_t * current_episode : instance_data) {
            const_instanceIt_t insIt_1 = current_episode->const_first();
            const_instanceIt_t insIt_2 = insIt_1;
            if(delay<0) {
                DEBUG_OUT(1,"Warning: Using negative delay");
                insIt_1 -= delay;
            } else {
                insIt_2 += delay;
            }
            while(insIt_1!=INVALID && insIt_2!=INVALID) {
                if(insIt_1->observation==s1) {
                    ++normalization;
                    if(insIt_2->observation==s2) {
                        prob += 1;
                    }
                }
                ++insIt_1;
                ++insIt_2;
            }
        }
    }
    // normalize and return
    if(normalization!=0) {
        return prob/normalization;
    } else {
        DEBUG_OUT(1,"Warning: Not enough data to determine probability");
        return 0;
    }
}

vector<DelayDistribution::probability_t> DelayDistribution::get_fixed_delay_probability_distribution(
    const observation_ptr_t& s1,
    const idx_t& delay
    ) {
    // initialize unnormalized probabilities
    vector<probability_t> prob;
    map<observation_ptr_t,idx_t> idx_map;
    idx_t observation_idx = 0;
    for(observation_ptr_t observation : *observation_space) {
        prob.push_back(0);
        idx_map[observation] = observation_idx;
        ++observation_idx;
    }
    size_t normalization = 0;

    // iterate through instances
    if(number_of_data_points>0) {
        for(instance_t * current_episode : instance_data) {
            const_instanceIt_t insIt_1 = current_episode->const_first();
            const_instanceIt_t insIt_2 = insIt_1;
            if(delay<0) {
                DEBUG_OUT(1,"Warning: Using negative delay");
                insIt_1 -= delay;
            } else {
                insIt_2 += delay;
            }
            while(insIt_1!=INVALID && insIt_2!=INVALID) {
                if(insIt_1->observation==s1) {
                    ++normalization;
                    prob[idx_map[insIt_2->observation]] += 1;
                }
                ++insIt_1;
                ++insIt_2;
            }
        }
    }

    // normalize and return
    if(normalization!=0) {
        for( auto& p : prob ) {
            p/=normalization;
        }
    } else {
        DEBUG_OUT(1,"Warning: Not enough data to determine probability");
    }
    return prob;
}

void DelayDistribution::get_delay_distribution(
    const observation_ptr_t& s1,
    const observation_ptr_t& s2,
    vector<probability_t> & forward,
    vector<probability_t> & backward,
    const idx_t& max_delay
    ) {

    //---------------------------------------------------//
    // handle identical observations and initialize containers //
    //---------------------------------------------------//
    if(s1==s2) {
        DEBUG_OUT(1,"Warning: Observations given to calculate delay distribution are identical");
        forward.assign(1,1);
        backward.assign(1,1);
        return;
    } else {
        forward.assign(1,0);
        backward.assign(1,0);
    }

    //-------------------//
    // count occurrences //
    //-------------------//
    size_t counter = 0;

    // iterate through episodes
    for(instance_t * current_episode : instance_data) {

        // iterate through current episode -- first time
        idx_t idx_1 = 0;
        for(const_instanceIt_t instance_1=current_episode->const_first(); instance_1!=INVALID; ++instance_1) {

            // iterate through current episode -- second time
            idx_t idx_2 = 0;
            for(const_instanceIt_t instance_2=current_episode->const_first(); instance_2!=INVALID; ++instance_2) {

                // determine delay and handel max_delay requirements
                idx_t delay = idx_1 - idx_2;
                if(max_delay>=0 && abs(delay)>max_delay) {
                    if(delay>0) {
                        continue;
                    } else {
                        break;
                    }
                }

                // update if observations match
                if(instance_1->observation==s1 && instance_2->observation==s2) {
                    if(delay==0) {
                        DEBUG_DEAD_LINE; // identical observations should be handeld above
                    } else if(delay<0) {
                        if((idx_t)backward.size()<1-delay) {
                            backward.resize(1-delay,0);
                        }
                        backward[-delay] += 1;
                    } else {
                        if((idx_t)forward.size()<1+delay) {
                            forward.resize(1+delay,0);
                        }
                        forward[-delay] += 1;
                    }
                    ++counter;
                }

                ++idx_2;
            }

            ++idx_1;
        }
    }

    //--------------------------------//
    // normalize to get probabilities //
    //--------------------------------//
    for( auto& prob : forward ) {
        prob /= counter;
    }
    for( auto& prob : backward ) {
        prob /= counter;
    }
}

DelayDistribution::probability_t DelayDistribution::get_mediator_probability(
    const observation_ptr_t& s1,
    const observation_ptr_t& s2,
    const observation_ptr_t& s3,
    const int& max_window
    ) {
    //-------------------//
    // count occurrences //
    //-------------------//
    probability_t prob = 0;
    size_t counter = 0;
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t ins_1=current_episode->first(); ins_1!=INVALID; ++ins_1 ) {
            if(ins_1->observation!=s1) { continue; }
            idx_t window = 1;
            for(const_instanceIt_t ins_3=ins_1+2; ins_3!=INVALID; ++ins_3 , ++window) {
                if(ins_3->observation!=s3 || (max_window>0 && window>max_window)) { continue; }
                for(const_instanceIt_t ins_2=ins_1+1; (const instance_t *)ins_2!=(const instance_t *)ins_3; ++ins_2 ) {
                    if(ins_2->observation==s2) {
                        prob += 1;
                    }
                    ++counter;
                }
            }
        }
    }

    return counter!=0 ? prob/counter : 0;
}

void DelayDistribution::get_pairwise_delay_distribution(
    pair_dist_map_t & dist_map,
    const int& max_dt
    ) {

    // clear map
    dist_map.clear();

    // fill map
    for(instance_t * current_episode : instance_data) {
        for(const_instanceIt_t ins_1=current_episode->first(); ins_1!=INVALID; ++ins_1 ) {
            idx_t dt = 1;
            for(const_instanceIt_t ins_2=ins_1+1; ins_2!=INVALID && (max_dt==-1 || dt<=max_dt); ++ins_2, ++dt) {
                // increment counts, perhaps explicitely call constructor when
                // element does not exist? (relies on default constructor
                // int()==0 for initialization)
                dist_map[make_tuple(ins_1->observation, ins_2->observation, dt)] += 1;
                dist_map[make_tuple(ins_1->observation, ins_2->observation, 0)] += 1;
            }
        }
    }

    // normalize
    for( auto el=dist_map.begin(); el!=dist_map.end(); ++el ) {
        auto tup = el->first;
        if(get<2>(tup)!=0) {
            el->second /= dist_map[make_tuple(get<0>(tup),get<1>(tup),0)];
        }
    }
}
