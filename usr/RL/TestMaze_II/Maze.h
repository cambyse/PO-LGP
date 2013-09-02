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
#include "util.h"

#include "debug.h"

class Maze {
public:

    USE_CONFIG_TYPEDEFS;

    typedef std::tuple<double,double,double> color_t;
    typedef std::vector<color_t>             color_vector_t;

    Maze(const double& eps = 0);
    virtual ~Maze();

    /** \brief State, which has 2D semantic, used by the Maze class. */
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

    /** \brief Renders the complete maze. */
    void render_initialize(QGraphicsView * v);

    /** \brief Updates the graphical representation. */
    void render_update(const color_vector_t * color = nullptr);

    /** \brief Perform a transition by executing an action. */
    void perform_transition(const action_t& action);

    /** \brief Perform a transition by executing an action and return resulting
     * state and reward by reference. */
    void perform_transition(const action_t& a, state_t& final_state, reward_t& r );

    /** \brief Returns the transition probability. */
    probability_t get_prediction(const instance_t*, const action_t&, const state_t&, const reward_t&, const bool& debug = false) const;

    /** \brief Validates a model by performing random transitions and comparing
     * the result to the model predicitons. */
    template < class Model >
    probability_t validate_model(
            const Model& model,
            size_t samples,
            probability_t * mean_model_likelihood,
            probability_t * mean_maze_likelihood
    );

    /** \brief Set epsilon to a value in [0,1]. */
    void set_epsilon(const double& e);

    /** \brief Get epsilon. */
    double get_epsilon() const { return epsilon; }

    /** \brief Set the current state of the agent. */
    void set_current_state(const state_t&);

    /** \brief Get a string describing all rewards. */
    static std::string get_rewards();

    /** \brief Get a string describing all walls. */
    static std::string get_walls();

private:

    //==========//
    // Typedefs //
    //==========//

    /** \brief Types of keys for opening doors. */
    enum KEY_TYPE {
        PASS_BUTTON, ///< Simply pass the field.
        STAY_BUTTON, ///< Perform STAY action on the field.
        UP_BUTTON,   ///< Perform UP action on the field.
        DOWN_BUTTON, ///< Perform DOWN action on the field.
        LEFT_BUTTON, ///< Perform LEFT action on the field.
        RIGHT_BUTTON ///< Perform RIGHT action on the field.
    };
    typedef std::tuple<MazeState, MazeState, MazeState, KEY_TYPE, idx_t, color_t> door_t;

    /* \brief Enum to identify the components for defining a reward. */
    enum REWARD_COMPONENTS {
        ACTIVATION_STATE, ///< Index of state where the reward is activated.
        RECEIVE_STATE,    ///< Index of state where the reward is received.
        TIME_DELAY,       ///< Index of time delay between activation and reception.
        REWARD_VALUE,     ///< Index of value of reward.
        ACTIVATION_TYPE,  ///< Index of activation type.
        R,                ///< Index of red component in [0,255] for displaying the reward.
        G,                ///< Index of green component in [0,255] for displaying the reward.
        B                 ///< Index of blue component in [0,255] for displaying the reward.
    };

    /** \brief Type of reward activation. */
    enum REWARD_ACTIVATION_TYPE {
        /** \brief Each time the agent passes the activation state a reward is
         * activated. */
        EACH_TIME_NO_PUNISH,
        /** \brief Only the last pass over the activation state activates a
         * reward. */
        ON_RELEASE_NO_PUNISH,
        /** \brief Each time the agent passes the activation state a reward is
         * activated. The agent receives the negative reward if it does not
         * collect to reward (stowaway scenario). */
        EACH_TIME_PUNISH_FAILURE,
        /** \brief Only the last pass over the activation state activates a
         * reward. The agent receives the negative reward if it does not collect
         * to reward (stowaway scenario). */
        ON_RELEASE_PUNISH_FAILURE
    };
    typedef std::vector<double> maze_reward_t;

    typedef std::vector<idx_t> wall_t;

    //==============//
    // Data Members //
    //==============//

    /** \brief The current state of the maze including the complete past. */
    instance_t * current_instance;

    QGraphicsView * view;                            ///< QGraphicsView to render the maze;

    double epsilon;                                  ///< Amount of stochasticity in transitions.
    MazeState current_state;                         ///< Current state of the agent in the maze.
    QGraphicsSvgItem *agent;                         ///< Svg image for rendering the agent.
    QGraphicsLineItem *action_line;                  ///< Line showing the last action.
    QGraphicsEllipseItem *action_point;              ///< Circle showing the last position for showing the last action.
    std::vector<QGraphicsItem*> borders;             ///< Graphic items for rendering the maze borders.
    std::vector<QGraphicsRectItem*> state_rects;     ///< Graphic items containing the state rects for rendering the states.

    static const std::vector<wall_t> walls;          ///< Defines the walls.
    static const std::vector<maze_reward_t> rewards; ///< Defines the rewards.
    static const std::vector<door_t> doors;          ///< Defines the doors.

    //==================//
    // Member Functions //
    //==================//

    /** \brief Add a frame around the maze in the graphics.
     *
     * Multiple frames with different colors encode the reward received in the
     * last time step.*/
    void frame_maze();

    /** \brief Add a state in the graphics. */
    void render_state(state_t s);

    /** \brief Add a wall in the graphics. */
    void render_wall(wall_t);

    /** \brief Add a door in the graphics. */
    void render_door(door_t);

    /** \brief Add a reward in the graphics. */
    void render_reward(maze_reward_t);

    /*! \brief Rescale the scene to fit into view. */
    void rescale_scene(QGraphicsView * view);

    /** \brief Simple helper function. */
    static int clamp(const int& lower, const int& upper, const int& value) {
        return util::clamp<int>(lower,upper,value);
    }

    /** \brief Returns the reward activation type as string. */
    static const char* reward_activation_type_str(REWARD_ACTIVATION_TYPE);
};

template < class Model >
Maze::probability_t Maze::validate_model(
        const Model& model,
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
        probability_t p_model = model.get_prediction(last_instance,action,state,reward);
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
