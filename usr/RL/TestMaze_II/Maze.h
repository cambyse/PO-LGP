#ifndef MAZE_H_
#define MAZE_H_

#include <QGraphicsView>
#include <QGraphicsSvgItem>
#include <map>
#include <tuple>
#include <deque>
#include <vector>
#include <string>
#include <sstream>

#include "Config.h"
#include "Representation/Representation.h"

#include "debug.h"

class Maze {
public:

    USE_CONFIG_TYPEDEFS;
    USE_REPRESENTATION_TYPEDEFS;

    typedef std::tuple<double,double,double> color_t;
    typedef std::vector<color_t>             color_vector_t;

    Maze(const double& eps = 0);

    virtual ~Maze();

    class MazeState {
    public:
    MazeState(const int& idx = 0): index(idx) {}
    MazeState(const int& x, const int& y): index(x+Config::maze_x_size*y) {}
        bool operator==(const MazeState& other) const { return this->index==other.index; }
        bool operator!=(const MazeState& other) const { return !((*this)==other); }
        bool operator<(const MazeState& other) const { return this->index<other.index; }
        idx_t state_idx() const { return index; }
        idx_t x() const { return index%Config::maze_x_size; }
        idx_t y() const { return index/Config::maze_x_size; }
        std::string print() const {
            std::stringstream ss;
            ss << "(" << x() << "," << y() << ")";
            return ss.str();
        }
        friend std::ostream& operator<<(std::ostream &out, const MazeState& s) {
            return (out << s.print());
        }
    private:
        idx_t index;
    };

    void render_initialize(QGraphicsView * view); ///< Renders the complete maze.
    void render_update(QGraphicsView * view, const color_vector_t * color = nullptr);

    void perform_transition(const action_t& action);
    void perform_transition(const action_t& a, state_t& final_state, reward_t& r );

    probability_t get_prediction(const instance_t*, const action_t&, const state_t&, const reward_t&) const;
    probability_t (Maze::*get_prediction_ptr())(const instance_t*, const action_t&, const state_t&, const reward_t&) const {
        return &Maze::get_prediction;
    }

    template < class Model >
    probability_t validate_model(
            const Model& model,
            probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const,
            size_t samples,
            probability_t * mean_model_likelihood,
            probability_t * mean_maze_likelihood
    );

    void set_epsilon(const double& e);
    double get_epsilon() const { return epsilon; }

    void set_current_state(const state_t&);

    static std::string get_rewards();
    static std::string get_walls();

private:

//    bool reward_active;
    instance_t * current_instance;
    static const double state_size;
    static const double wall_width;
    static const double reward_start_size;
    static const double reward_end_size;
    static const double reward_end_ratio;
    static const double text_scale;
    static const double text_center;
    double epsilon;
    MazeState current_state;
//    MazeState button_state, smiley_state;
//    QGraphicsSvgItem *button, *smiley;
    QGraphicsSvgItem *agent;
    std::vector<QGraphicsItem*> borders;
    std::vector<QGraphicsRectItem*> state_rects;

    static const std::vector<std::vector<idx_t> > walls;
    static const std::vector<std::vector<idx_t> > rewards;
    enum REWARD_COMPONENTS { ACTIVATION_STATE, RECEIVE_STATE, TIME_DELAY, REWARD_IDX, ACTIVATION_TYPE, R, G, B };
    enum REWARD_ACTIVATION { EACH_TIME, ON_RELEASE };

    /*! \brief Rescale the scene to fit into view. */
    void rescale_scene(QGraphicsView * view);

    static int clamp(const int& lower, const int& upper, const int& value) {
        if(value<lower) {
            return lower;
        } else if(value>upper) {
            return upper;
        }
        return value;
    }
};

template < class Model >
Maze::probability_t Maze::validate_model(
        const Model& model,
        probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const,
        size_t samples,
        probability_t * mean_model_likelihood,
        probability_t * mean_maze_likelihood
) {
    probability_t kl_divergence = 0;
    *mean_model_likelihood = 0;
    *mean_maze_likelihood = 0;
    for(size_t transition_counter=0; transition_counter<samples; ++transition_counter) {
        action_t action = action_t::random_action();
        state_t state;
        reward_t reward;
        instance_t * last_instance = current_instance;
        perform_transition(action,state,reward);
        probability_t p_maze = get_prediction(last_instance,action,state,reward);
        probability_t p_model = (model.*prediction)(last_instance,action,state,reward);
        kl_divergence += log(p_maze/p_model);
        *mean_model_likelihood += p_model;
        *mean_maze_likelihood += p_maze;
    }
    *mean_model_likelihood/=samples;
    *mean_maze_likelihood/=samples;
    return kl_divergence/samples;
}

#include "debug_exclude.h"

#endif /* MAZE_H_ */
