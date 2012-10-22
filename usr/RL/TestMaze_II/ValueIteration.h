/*
 * ValueIteration.h
 *
 *  Created on: Oct 22, 2012
 *      Author: robert
 */

#ifndef VALUEITERATION_H_
#define VALUEITERATION_H_

#include <map>

template< class P, class R, class State, class Action, class Value = double>
class ValueIteration
{

public:

    typedef std::map<State,Value> ValueMap;

    ValueIteration(P const &probability, R const &reward, double& g = 0.9):
        p(probability),
        r(reward),
        gamma(g),
        value_map(NULL),
        const_value_map(NULL) {
        value_map = new ValueMap();
        const_value_map = new ValueMap();
    }
    virtual ~ValueIteration() {}

    double value(State &s) { return (*const_value_map)[s]; }

    void update() {
        ValueMap * tmp_value_map = const_value_map;
        const_value_map = value_map;
        value_map = tmp_value_map;
        for(State state_from=State::start(); state_from==State::stop(); ++state_from) {
            (*value_map)[state_from] = 0;
            for(Action action=Action::start(); action==Action::stop(); ++action) {
                for(State state_to=State::start(); state_to==State::stop(); ++state_to) {
                    (*value_map)[state_from] += p(state_to, state_from, action) * ( r(state_to, action) + gamma*value(state_to));
                }
            }
        }
    }

private:
    double gamma;
    P const &p;
    R const &r;
    ValueMap * value_map;
    ValueMap const * const_value_map;

};

#endif /* VALUEITERATION_H_ */
