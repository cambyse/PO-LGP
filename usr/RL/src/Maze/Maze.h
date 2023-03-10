#ifndef MAZE_H_
#define MAZE_H_

#include <environment/PredictiveEnvironment.h>
#include <environment/Visualizer.h>

#include <QGraphicsSvgItem>
#include <map>
#include <deque>
#include <string>
#include <sstream>

#include <config/Config.h>
#include <util/util.h>
#include <representation/Feature.h>

#include "MazeObservation.h"
#include "MazeAction.h"
#include <representation/ListedReward.h>

class Maze: public PredictiveEnvironment, public Visualizer {
public:

    USE_CONFIG_TYPEDEFS;
    typedef MazeAction action_t;
    typedef MazeObservation observation_t;
    typedef ListedReward reward_t;

    using PredictiveEnvironment::perform_transition; // so that lookup works

    Maze(const double& eps = 0, const QString& s = "Default");
    virtual ~Maze() override;

    /** \brief Set a maze by name. */
    virtual bool set_maze(const QString& s);

    /** \brief Renders the complete maze. */
    virtual void render_initialize(QGraphicsView * v) override;

    /** \brief Updates the graphical representation. */
    virtual void render_update() override;

    /** \brief Clears the scene used by Visualizer::view and resets all
     * pointers to nullptr. */
    virtual void render_tear_down() override;

    void set_state_colors(const color_vector_t colors = color_vector_t());

    void show_distribution(const std::vector<probability_t> dist, bool scale_as_sqrt = false);

    /** \brief Perform a transition by executing an action and return which rewards
     * were active. */
    virtual void perform_transition(const action_ptr_t& a, std::vector<std::pair<int,int> > * reward_vector);

    /** \brief Perform a transition by executing an action. */
    virtual void perform_transition(const action_ptr_t& action) override;

    /** \brief Returns the transition probability. */
    probability_t get_prediction(const_instance_ptr_t, const action_ptr_t&, const observation_ptr_t&, const reward_ptr_t&) const override;

    /** \brief Returns the transition probability and which rewards were active.
     *
     * The first counter in each pair counts positive rewards, the second
     * punishments (for not collecting an activated reward). */
    probability_t get_prediction(const_instance_ptr_t, const action_ptr_t&, const observation_ptr_t&, const reward_ptr_t&, std::vector<std::pair<int,int> > * reward_vector) const;

    virtual void get_features(f_set_t & basis_features, FeatureLearner::LEARNER_TYPE type) const override;

    void print_reward_activation_on_random_walk(const int& walk_length);

    /** \brief Set epsilon to a value in [0,1]. */
    void set_epsilon(const double& e);

    /** \brief Get epsilon. */
    double get_epsilon() const { return epsilon; }

    /** \brief Set the current state of the agent. */
    void set_current_observation(const observation_ptr_t&);

    static const std::vector<QString> get_maze_list();

    /** \brief Get a string describing all rewards. */
    virtual std::string get_rewards();

    /** \brief Get a string describing all walls. */
    virtual std::string get_walls();

    /** \brief Get a string describing all doors. */
    virtual std::string get_doors();

    virtual void print_transition(action_ptr_t& a, observation_ptr_t& o, reward_ptr_t& r) const;

    /** \brief Checks if all observations and states used in the current maze
     * definition are valid. */
    bool check_maze_definition() const;

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
        REWARD_VALUE,            ///< Value of reward.
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
    observation_t current_observation;               ///< Current state of the agent in the maze.
    double epsilon;                                  ///< Amount of stochasticity in transitions.
    QGraphicsSvgItem *agent;                         ///< Svg image for rendering the agent.
    QGraphicsLineItem *action_line;                  ///< Line showing the last action.
    QGraphicsEllipseItem *action_point;              ///< Circle showing the last position for showing the last action.
    std::vector<QGraphicsItem*> borders;             ///< Graphic items for rendering the maze borders.
    std::vector<QGraphicsRectItem*> state_rects;     ///< Graphic items containing the state rects for rendering the states.
    color_vector_t state_colors;                     ///< Color vector for all states.

    maze_t                      current_maze;        ///< Currently used maze.
    static const std::vector<maze_t> maze_list;      ///< List of all mazes.
    std::vector<wall_t>         walls;               ///< Defines the walls.
    std::vector<maze_reward_t>  rewards;             ///< Defines the rewards.
    std::vector<door_t>         doors;               ///< Defines the doors.

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

#endif /* MAZE_H_ */
