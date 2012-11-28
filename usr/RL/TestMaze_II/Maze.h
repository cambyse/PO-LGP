/*
 * Maze.h
 *
 *  Created on: Oct 10, 2012
 *      Author: robert
 */

#ifndef MAZE_H_
#define MAZE_H_

#include <QtGui/QGraphicsView>
#include <QtSvg/QGraphicsSvgItem>
#include <map>
#include <tuple>
#include <deque>
#include <vector>

#include "Data.h"

class Maze {
public:

    typedef Data::action_t        action_t;
    typedef Data::state_t         state_t;
    typedef Data::reward_t        reward_t;
    typedef Data::input_data_t    input_data_t;
    typedef Data::output_data_t   output_data_t;

    class MazeState {
    public:
        MazeState(const int& idx = 0): index(idx) {}
        MazeState(const int& x, const int& y, const int& x_dim): index(x+x_dim*y) {}
        bool operator==(const MazeState& other) const { return this->index==other.index; }
        bool operator!=(const MazeState& other) const { return !((*this)==other); }
        bool operator<(const MazeState& other) const { return this->index<other.index; }
        int idx() const { return index; }
        int x(const int& x_dim) const { return index%x_dim; }
        int y(const int& x_dim) const { return index/x_dim; }
    private:
        int index;
    };

    Maze(const int& x_dimension, const int& y_dimension, const double& eps = 0);

    virtual ~Maze();

    void render_initialize(QGraphicsView * view); ///< Renders the complete maze.
    void render_update(QGraphicsView * view);

    void perform_transition(const action_t& action);
    void perform_transition(const action_t& a, Data::state_t& final_state, reward_t& r );

//    template< class TransitionProbabilities >
//    void initialize_transition_probabilities(TransitionProbabilities&);
//
//    template< class RewardProbabilities >
//    void initialize_reward_probabilities(RewardProbabilities&);

    void set_time_delay(const int& new_time_delay);
    int get_time_delay() { return time_delay; }

    int number_of_states() const { return x_dim*y_dim; }

    void set_epsilong(const double& e);
    double get_epsilong() const { return epsilon; }

private:

    int x_dim; ///< x dimension.
    int y_dim; ///< y dimension.
    int time_delay;
    static const double state_size;
    std::deque<bool> reward_timer;
    double epsilon;
    MazeState current_state;
    std::map< std::tuple<MazeState,action_t>, std::vector<std::tuple<MazeState,double> > > transition_map;
    MazeState button_state, smiley_state;
    QGraphicsSvgItem *agent, *button, *smiley;

    void set_current_state(const MazeState& s);

    /*! \brief Rescale the scene to fit into view. */
    void rescale_scene(QGraphicsView * view);

    /*! \brief Initialize all transitions. */
    void create_transitions();

    int clamp(const int& lower, const int& upper, const int& value) {
        if(value<lower) {
            return lower;
        } else if(value>upper) {
            return upper;
        }
        return value;
    }
};

//template< class TransitionProbabilities >
//void Maze::initialize_transition_probabilities(TransitionProbabilities& transition_probabilities) {
//    for(Data::state_t state_from=0; state_from<x_dim*y_dim; ++state_from) {
//        for(Data::action_t action = 0; action<Data::action_n; ++action) {
//            std::vector<std::tuple<MazeState,double> > state_vector = transition_map[std::make_tuple(state_from,action)];
//            for(uint idx=0; idx<state_vector.size(); ++idx) {
//                transition_probabilities.set_transition_probability(state_from,action,Data::state_t(std::get<0>(state_vector[idx]).idx()),std::get<1>(state_vector[idx]));
//            }
//        }
//    }
//}

//template< class RewardProbabilities >
//void Maze::initialize_reward_probabilities(RewardProbabilities& reward_probabilities) {
//    reward_probabilities.set_reward_probability();
//}

#endif /* MAZE_H_ */
