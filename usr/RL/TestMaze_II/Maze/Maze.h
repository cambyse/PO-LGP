#ifndef MAZE_H_
#define MAZE_H_

#include "../VisualEnvironment.h"
#include "../PredictiveEnvironment.h"

#include <QGraphicsSvgItem>
#include <map>
#include <deque>
#include <string>
#include <sstream>

#include "../Config.h"
#include "../util.h"
#include "../Feature.h"

#include "MazeObservation.h"
#include "MazeAction.h"
#include "../ListedReward.h"

#include "../debug.h"

class Maze: public VisualEnvironment, public PredictiveEnvironment {
public:

    USE_CONFIG_TYPEDEFS;
    typedef Feature::const_feature_ptr_t f_ptr_t;
    typedef MazeAction action_t;
    typedef MazeObservation observation_t;
    typedef ListedReward reward_t;

    Maze(const double& eps = 0);
    virtual ~Maze();

    /** \brief Set a maze by name. */
    virtual void set_maze(const QString& s);

    /** \brief Renders the complete maze. */
    virtual void render_initialize(QGraphicsView * v) override;

    /** \brief Updates the graphical representation. */
    virtual void render_update() override;

    void set_state_colors(const color_vector_t colors = color_vector_t());

    /** \brief Perform a transition by executing an action. */
    void perform_transition(const action_ptr_t& action);

    /** \brief Perform a transition by executing an action and return resulting
     * observation and reward by reference. */
    void perform_transition(const action_ptr_t& a, observation_ptr_t& final_observation, reward_ptr_t& r );

    /** \brief Perform a transition by executing an action and return which rewards
     * were active. */
    void perform_transition(const action_ptr_t& a, std::vector<std::pair<int,int> > * reward_vector);

    /** \brief Returns the transition probability. */
    probability_t get_prediction(const instance_t*, const action_ptr_t&, const observation_ptr_t&, const reward_ptr_t&) const override;

    /** \brief Returns the transition probability and which rewards were active.
     *
     * The first counter in each pair counts positive rewards, the second
     * punishments (for not collecting an activated reward). */
    probability_t get_prediction(const instance_t*, const action_ptr_t&, const observation_ptr_t&, const reward_ptr_t&, std::vector<std::pair<int,int> > * reward_vector) const;

    virtual void get_features(std::vector<f_ptr_t> & basis_features, FeatureLearner::LEARNER_TYPE type) const override;

    void print_reward_activation_on_random_walk(const int& walk_length);

    /** \brief Set epsilon to a value in [0,1]. */
    void set_epsilon(const double& e);

    /** \brief Get epsilon. */
    double get_epsilon() const { return epsilon; }

    /** \brief Set the current state of the agent. */
    void set_current_observation(const observation_ptr_t&);

    const instance_t * get_current_instance() const { return current_instance; }

    /** \brief Get a string describing all rewards. */
    std::string get_rewards();

    /** \brief Get a string describing all walls. */
    std::string get_walls();

    /** \brief Get a string describing all doors. */
    std::string get_doors();

private:

    //==========//
    // Typedefs //
    //==========//

    /* \brief Enum to identify the components for defining a door. */
    enum DOOR_COMPONENTS {
        DOOR_STATE_FROM,
        DOOR_STATE_TO,
        DOOR_KEY_STATE,
        DOOR_KEY,
        DOOR_TIME_DELAY,
        DOOR_COLOR
    };

    /** \brief Types of keys for opening doors. */
    enum KEY_TYPE {
        PASS_BUTTON, ///< Simply pass the field.
        STAY_BUTTON, ///< Perform STAY action on the field.
        UP_BUTTON,   ///< Perform UP action on the field.
        DOWN_BUTTON, ///< Perform DOWN action on the field.
        LEFT_BUTTON, ///< Perform LEFT action on the field.
        RIGHT_BUTTON ///< Perform RIGHT action on the field.
    };
    typedef std::tuple<observation_t, observation_t, observation_t, KEY_TYPE, idx_t, color_t> door_t;

    /* \brief Enum to identify the components for defining a reward. */
    enum REWARD_COMPONENTS {
        REWARD_ACTIVATION_STATE, ///< Index of state where the reward is activated.
        REWARD_RECEIVE_STATE,    ///< Index of state where the reward is received.
        REWARD_TIME_DELAY,       ///< Index of time delay between activation and reception.
        REWARD_VALUE,            ///< Index of value of reward.
        REWARD_ACTIVATION,       ///< Index of activation type.
        REWARD_R,                ///< Index of red component in [0,255] for displaying the reward.
        REWARD_G,                ///< Index of green component in [0,255] for displaying the reward.
        REWARD_B                 ///< Index of blue component in [0,255] for displaying the reward.
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

    struct maze_t {
        maze_t(const QString & n,
               const int kk,
               const action_ptr_t & a,
               const observation_ptr_t & o,
               const reward_ptr_t & r,
               const std::vector<wall_t> & w,
               const std::vector<maze_reward_t> & mr,
               const std::vector<door_t> & d):
            name(n), k(kk),
            action_space(a), observation_space(o), reward_space(r),
            walls(w), rewards(mr), doors(d)
        {}
        ~maze_t() = default;
        QString                    name;
        int                        k;
        action_ptr_t               action_space;
        observation_ptr_t          observation_space;
        reward_ptr_t               reward_space;
        std::vector<wall_t>        walls;
        std::vector<maze_reward_t> rewards;
        std::vector<door_t>        doors;
    };

    //==============//
    // Data Members //
    //==============//

    int k;                             ///< k-MDP length.

    /** \brief The current state of the maze including the complete past. */
    instance_t * current_instance;
    observation_t current_observation;               ///< Current state of the agent in the maze.
    double epsilon;                                  ///< Amount of stochasticity in transitions.
    QGraphicsSvgItem *agent;                         ///< Svg image for rendering the agent.
    QGraphicsLineItem *action_line;                  ///< Line showing the last action.
    QGraphicsEllipseItem *action_point;              ///< Circle showing the last position for showing the last action.
    std::vector<QGraphicsItem*> borders;             ///< Graphic items for rendering the maze borders.
    std::vector<QGraphicsRectItem*> state_rects;     ///< Graphic items containing the state rects for rendering the states.
    color_vector_t state_colors;                     ///< Color vector for all states.

    static const std::vector<maze_t> maze_list;      ///< List of all mazes.
    static std::vector<wall_t>         walls;               ///< Defines the walls.
    static std::vector<maze_reward_t>  rewards;             ///< Defines the rewards.
    static std::vector<door_t>         doors;               ///< Defines the doors.

    //==================//
    // Member Functions //
    //==================//

    /** \brief Add a frame around the maze in the graphics.
     *
     * Multiple frames with different colors encode the reward received in the
     * last time step.*/
    void frame_maze();

    /** \brief Add a state in the graphics. */
    void render_state(observation_ptr_t observation);

    /** \brief Add a wall in the graphics. */
    void render_wall(wall_t);

    /** \brief Add a door in the graphics. */
    void render_door(door_t);

    /** \brief Add a reward in the graphics. */
    void render_reward(maze_reward_t);

    /** \brief Simple helper function. */
    static int clamp(const int& lower, const int& upper, const int& value) {
        return util::clamp<int>(lower,upper,value);
    }

    /** \brief Returns the reward activation type as string. */
    static const char* reward_activation_type_str(REWARD_ACTIVATION_TYPE);
};

#include "../debug_exclude.h"

#endif /* MAZE_H_ */
