#include "Maze.h"

#include "../util/QtUtil.h"
#include "../util/ColorOutput.h"

#define DEBUG_LEVEL 0
#include "../debug.h"

#include <algorithm>

using std::tuple;
using std::pair;
using std::make_pair;
using std::get;
using std::vector;
using std::get;
using std::string;
using std::stringstream;
using std::endl;
using std::min;
using std::max;

using util::INVALID;

static const double state_size = 0.9;                             // Size of states for rendering.
static const double wall_width = 0.08;                            // Width of walls for rendering.
static const double reward_start_size = 0.15;                     // Size of reward start marker for rendering.
static const double reward_end_size = 0.2;                        // Size of reward end marker for rendering.
static const double reward_end_ratio = 0.5;                       // Length-to-width ratio of reward end marker (arrow) for rendering.
static const double reward_end_notch = 0.25;                      // Depth of the arrow notch.
static const double text_scale = 0.008;                           // Scale factor for text size.
static const QFont  text_font = QFont("Helvetica [Cronyx]", 12);  // Font for texts.
static const double text_center = 0.3;                            // How close the text should be positioned to the midpoint between start and end marker.
static const double border_margin = 0.4*state_size;               // Margin for drawing the border around the maze.
static const double action_line_length_factor = 0.8;              // How long the action line is relative to the state size.
static const double action_point_size_factor = 0.5;               // How large the action point is relative to the state size.
static const bool draw_text = false;                              // Whether to draw texts.

const vector<Maze::maze_t> Maze::maze_list = {
#include "Maze_Default.cpp"
    ,
#include "Maze_Minimal.cpp"
    ,
#include "Maze_3x3.cpp"
    ,
#include "Maze_4x4_I.cpp"
    ,
#include "Maze_4x4_II.cpp"
    ,
#include "Maze_4x4_III.cpp"
    ,
#include "Maze_6x6.cpp"
    ,
#include "Maze_10x10.cpp"
    ,
#include "Maze_Markov.cpp"
//     ,
// #include "Maze_Wrong.cpp" // for testing unit tests
};
Maze::Maze(const double& eps, const QString& s):
    PredictiveEnvironment(action_ptr_t(), observation_ptr_t(), reward_ptr_t()), // set by calling set_maze in body
    current_observation(*(maze_list[0].observation_space.get_derived<observation_t>())),
    epsilon(eps),
    agent(nullptr),
    current_maze(maze_list[0])
{
    // set a maze
    set_maze(s);
}

Maze::~Maze() {
    delete agent;
    delete current_instance;
}

bool Maze::set_maze(const QString& s) {

    // maze to use (defaults to first in list)
    current_maze = maze_list[0];

    // find maze
    bool found = false;
    for(maze_t maze : maze_list) {
        if(maze.name==s) {
            current_maze = maze;
            found = true;
            break;
        }
    }

    if(!found) {
        DEBUG_ERROR("Could not find maze with name '" << s << "'");
    }

    // set the maze
    k                 = current_maze.k;
    action_space      = current_maze.action_space;
    observation_space = current_maze.observation_space;
    reward_space      = current_maze.reward_space;
    walls             = current_maze.walls;
    rewards           = current_maze.rewards;
    doors             = current_maze.doors;

    // set current observation (and instance)
    set_current_observation(observation_space);

    // set state colors to default
    set_state_colors();

    // check maze
    check_maze_definition();

    return found;
}

void Maze::render_initialize(QGraphicsView * v) {

    // intitialize view
    Visualizer::render_initialize(v);

    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();

    // Put Border Around Maze
    frame_maze();

    // Render States
    state_rects.clear();
    for(observation_ptr_t observation : observation_space) {
        render_state(observation);
    }

    // Render Walls
    for( auto w : walls ) {
        render_wall(w);
    }

    // Render Doors
    for( auto d : doors ) {
        render_door(d);
    }

    // Render action line and circle
    QPen action_line_pen(QColor(0,0,0,50),0.1,Qt::SolidLine,Qt::RoundCap);
    QPen action_point_pen(QColor(0,0,0),0.01,Qt::SolidLine,Qt::RoundCap);
    QBrush action_point_brush(QColor(0,0,0,30));
    action_line = scene->addLine(current_observation.get_x_pos(),current_observation.get_y_pos(),current_observation.get_x_pos(),current_observation.get_y_pos(),action_line_pen);
    double ap_size = state_size*action_point_size_factor;
    action_point = scene->addEllipse(current_observation.get_x_pos()-ap_size/2,current_observation.get_y_pos()-ap_size/2,ap_size,ap_size,action_point_pen,action_point_brush);

    // Rewards
    for(maze_reward_t reward : rewards ) {
        render_reward(reward);
    }

    // render agent
    {
        if(!agent) {
            delete agent;
        }
        agent = new QGraphicsSvgItem("Images/agent.svg");
        double scale = 0.2;
        agent->setScale(scale);
        QSizeF s = agent->boundingRect().size()*scale;
        agent->setPos(current_observation.get_x_pos()-s.width()/2, current_observation.get_y_pos()-s.height()/2);
        agent->setElementId("normal");
        scene->addItem(agent);
    }

    rescale_scene(view);
}


void Maze::render_update() {
    // set agent position and mirror to make 'stay' actions visible
    QSizeF s = agent->boundingRect().size();
    agent->setPos(current_observation.get_x_pos()-agent->scale()*s.width()/2, current_observation.get_y_pos()-agent->scale()*s.height()/2);
    agent->setElementId(agent->elementId()=="normal" ? "mirrored" : "normal");

    // set action line and circle
    observation_t last_state(*((current_instance->const_it()-1)->observation.get_derived<observation_t>()));
    double al_length = state_size*action_line_length_factor;
    double ap_size = state_size*action_point_size_factor;
    action_point->setRect(last_state.get_x_pos()-ap_size/2,last_state.get_y_pos()-ap_size/2,ap_size,ap_size);
    switch(current_instance->action.get_derived<action_t>()->get_action()) {
    case action_t::ACTION::STAY:
        action_line->setLine(last_state.get_x_pos(),last_state.get_y_pos(),last_state.get_x_pos(),last_state.get_y_pos());
        break;
    case action_t::ACTION::UP:
        action_line->setLine(last_state.get_x_pos(),last_state.get_y_pos(),last_state.get_x_pos(),last_state.get_y_pos()-al_length);
        break;
    case action_t::ACTION::DOWN:
        action_line->setLine(last_state.get_x_pos(),last_state.get_y_pos(),last_state.get_x_pos(),last_state.get_y_pos()+al_length);
        break;
    case action_t::ACTION::LEFT:
        action_line->setLine(last_state.get_x_pos(),last_state.get_y_pos(),last_state.get_x_pos()-al_length,last_state.get_y_pos());
        break;
    case action_t::ACTION::RIGHT:
        action_line->setLine(last_state.get_x_pos(),last_state.get_y_pos(),last_state.get_x_pos()+al_length,last_state.get_y_pos());
        break;
    default:
        DEBUG_ERROR("Invalid action (" << current_instance->action << ")");
    }

    // show reward by border color
    int border_idx = 0;
    bool matching_reward = false; // safety check
    for(reward_ptr_t reward : reward_space) {
        if(current_instance->reward==reward) {
            borders[border_idx]->setVisible(true);
            matching_reward = true;
        } else {
            borders[border_idx]->setVisible(false);
        }
        ++border_idx;
    }

    if(!matching_reward) {
        DEBUG_ERROR("Current reward is invalid (" << current_instance->reward << ")");
    }

    // Change State Color
    idx_t col_idx = 0;
    for( auto rect_ptr : state_rects ) {
        rect_ptr->setBrush( QColor( get<COLOR_R>(state_colors[col_idx])*255, get<COLOR_G>(state_colors[col_idx])*255, get<COLOR_B>(state_colors[col_idx])*255 ) );
        ++col_idx;
    }

    rescale_scene(view);
}

void Maze::render_tear_down() {
    view->scene()->clear();
    action_line = nullptr;
    action_point = nullptr;
    agent = nullptr;
    borders.assign(0,nullptr);
    state_rects.assign(0,nullptr);
}

void Maze::set_state_colors(const color_vector_t colors) {
    state_colors = colors;
    uint state_n = current_observation.get_x_dim()*current_observation.get_y_dim();
    if(state_colors.size()==0) {
        state_colors.assign(state_n, color_t(0.9,0.9,0.9));
    } else if(state_colors.size()!=state_n) {
        DEBUG_ERROR("Number of colors does not match number of states");
        state_colors.resize(state_n, color_t(0.9,0.9,0.9));
    }
}

void Maze::perform_transition(const action_ptr_t& action, std::vector<std::pair<int,int> > * reward_vector) {

    observation_t old_state = current_observation; // remember current (old) state

    if(DEBUG_LEVEL>=1) {
        DEBUG_OUT(1,"Current instance: ");
        const_instanceIt_t insIt = current_instance->it();
        for(idx_t k_idx=0; k_idx<k; ++k_idx) {
            DEBUG_OUT(1,"    " << (*insIt) );
            --insIt;
        }
    }

    // perform transition
    probability_t prob_threshold = drand48();
    DEBUG_OUT(2,"Prob threshold = " << prob_threshold);
    probability_t prob_accum = 0;
    bool was_set = false;
    for(observation_ptr_t observation_to : observation_space) {
        if(was_set) {
            break;
        }
        for(reward_ptr_t reward : reward_space) {
            if(was_set) {
                break;
            }
            probability_t prob = get_prediction(current_instance, action, observation_to, reward, reward_vector);
            DEBUG_OUT(2,"observation(" << observation_to << "), reward(" << reward << ") --> prob=" << prob);
            prob_accum += prob;
            if(prob_accum>prob_threshold) {
                current_observation = *(observation_to.get_derived<observation_t>());
                current_instance = current_instance->append_instance(action, observation_to, reward);
                was_set = true;
                DEBUG_OUT(2,"CHOOSE");
            }
        }
    }
    if(!was_set) {
        DEBUG_ERROR("Unnormalized probabilities [sum(p)=" << prob_accum << "]--> no transition performed." );
    }

    DEBUG_OUT(1, "(" <<
              old_state.get_x_pos() << "," <<
              old_state.get_y_pos() << ") + " <<
              action << " ==> (" <<
              current_observation.get_x_pos() << "," <<
              current_observation.get_y_pos() << ")"
        );
}

void Maze::perform_transition(const action_ptr_t& action) {
    perform_transition(action, nullptr);
}

Maze::probability_t Maze::get_prediction(
    const instance_t*        instance_from,
    const action_ptr_t&      action,
    const observation_ptr_t& observation_to,
    const reward_ptr_t&      reward
    ) const {
    return get_prediction(instance_from, action, observation_to, reward, nullptr);
}

Maze::probability_t Maze::get_prediction(
    const instance_t*        instance_from,
    const action_ptr_t&      action,
    const observation_ptr_t& observation_to,
    const reward_ptr_t&      reward,
    vector<pair<int,int> > * reward_vector
    ) const {

    // state 'from' and 'to'
    observation_t maze_state_from(*(instance_from->observation.get_derived<observation_t>()));
    observation_t maze_state_to(*(observation_to.get_derived<observation_t>()));
    int x_from = maze_state_from.get_x_pos();
    int y_from = maze_state_from.get_y_pos();

    // states that can in principle be reached (ignoring walls and doors)
    int x_dim = current_observation.get_x_dim();
    int y_dim = current_observation.get_y_dim();
    observation_t maze_state_stay(  x_dim, y_dim, x_from, y_from                     );
    observation_t maze_state_left(  x_dim, y_dim, clamp(0,x_dim-1,x_from-1), y_from  );
    observation_t maze_state_right( x_dim, y_dim, clamp(0,x_dim-1,x_from+1), y_from  );
    observation_t maze_state_up(    x_dim, y_dim, x_from, clamp(0,y_dim-1, y_from-1) );
    observation_t maze_state_down(  x_dim, y_dim, x_from, clamp(0,y_dim-1, y_from+1) );

    // check if state_to can be reached at all
    if( maze_state_to!=maze_state_stay  &&
        maze_state_to!=maze_state_left  &&
        maze_state_to!=maze_state_right &&
        maze_state_to!=maze_state_up    &&
        maze_state_to!=maze_state_down ) {
        return 0;
    }

    // check and remember blocking/unblocking (store references as array for
    // later manipulation, ignore 'stay' which cannot be blocked/unblocked)
    observation_t reachable_states[4] = { maze_state_left, maze_state_right, maze_state_up, maze_state_down };
    bool  left_unblocked_by_door = false;
    bool right_unblocked_by_door = false;
    bool    up_unblocked_by_door = false;
    bool  down_unblocked_by_door = false;
    bool* unblocked_by_door[4] = { &left_unblocked_by_door, &right_unblocked_by_door, &up_unblocked_by_door, &down_unblocked_by_door };
    bool  left_blocked_by_wall = false;
    bool right_blocked_by_wall = false;
    bool    up_blocked_by_wall = false;
    bool  down_blocked_by_wall = false;
    bool* blocked_by_wall[4] = { &left_blocked_by_wall, &right_blocked_by_wall, &up_blocked_by_wall, &down_blocked_by_wall };

    // check for doors (unblocking doors take precedence over closed doors)
    for( auto d : doors ) {
        observation_t s1 = get<DOOR_STATE_FROM>(d);
        observation_t s2 = get<DOOR_STATE_TO>(d);
        observation_t s3 = get<DOOR_KEY_STATE>(d);
        KEY_TYPE kt =  get<DOOR_KEY>(d);
        idx_t delay =  get<DOOR_TIME_DELAY>(d);

        // iterate through reachable state
        for(int idx=0; idx<4; ++idx) {
            observation_t tmp_maze_state_to = reachable_states[idx];

            // check if transition was unblocked already
            if(*unblocked_by_door[idx]) {
                continue;
            }

            // check if the door connects the two states
            if( (tmp_maze_state_to==s1 && maze_state_from==s2) || (tmp_maze_state_to==s2 && maze_state_from==s1) ) {

                // action must be set one iteration earlier because some times
                // (e.g. in look-ahead search tree) one cannot go back to the
                // future because it is not uniquely defined
                action_t past_action = *(action.get_derived<action_t>());

                // check if key was activated
                idx_t steps_in_past = 0;
                for(const_instanceIt_t insIt = instance_from->const_it();
                    insIt!=INVALID && steps_in_past<=abs(delay);
                    past_action=*(insIt->action.get_derived<action_t>()), --insIt, ++steps_in_past) {

                    // check if delay matches
                    if(delay<0 || steps_in_past==delay) { // exact match for positve delay less-equal match for negative

                        // check if key state was visited at that time
                        if(insIt->observation==s3) {

                            // check action
                            switch(kt) {
                            case PASS_BUTTON:
                                // action does not matter
                                *unblocked_by_door[idx]=true;
                                break;
                            case STAY_BUTTON:
                                if(past_action==(action_t)action_t::ACTION::STAY) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case UP_BUTTON:
                                if(past_action==(action_t)action_t::ACTION::UP) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case DOWN_BUTTON:
                                if(past_action==(action_t)action_t::ACTION::DOWN) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case LEFT_BUTTON:
                                if(past_action==(action_t)action_t::ACTION::LEFT) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case RIGHT_BUTTON:
                                if(past_action==(action_t)action_t::ACTION::RIGHT) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    // check for walls
    for(auto w : walls) {
        observation_t s1 = current_observation.new_observation(w[0]);
        observation_t s2 = current_observation.new_observation(w[1]);

        // iterate through reachable states
        for(int idx=0; idx<4; ++idx) {
            observation_t tmp_maze_state_to = reachable_states[idx];

            // ignore wall if transition was unblocked by door or already blocked by wall
            // if(*unblocked_by_door[idx] || *blocked_by_wall[idx]) {
            //     continue;
            // }

            // check if wall is between the two states
            if( (maze_state_from==s1 && tmp_maze_state_to==s2) || (maze_state_from==s2 && tmp_maze_state_to==s1) ) {
                *blocked_by_wall[idx] = true;
            }
        }
    }

    // determine effective states (by considering blocking/unblocking)
    observation_t effective_maze_state_stay  = maze_state_stay;
    observation_t effective_maze_state_left  = ( left_unblocked_by_door || ! left_blocked_by_wall) ? maze_state_left  : maze_state_stay;
    observation_t effective_maze_state_right = (right_unblocked_by_door || !right_blocked_by_wall) ? maze_state_right : maze_state_stay;
    observation_t effective_maze_state_up    = (   up_unblocked_by_door || !   up_blocked_by_wall) ? maze_state_up    : maze_state_stay;
    observation_t effective_maze_state_down  = ( down_unblocked_by_door || ! down_blocked_by_wall) ? maze_state_down  : maze_state_stay;

    // determine state transition probabilites via effective states
    probability_t prob = 0;
    {
        // random action
        if(maze_state_to==effective_maze_state_stay ) prob += epsilon/5;
        if(maze_state_to==effective_maze_state_left ) prob += epsilon/5;
        if(maze_state_to==effective_maze_state_right) prob += epsilon/5;
        if(maze_state_to==effective_maze_state_up   ) prob += epsilon/5;
        if(maze_state_to==effective_maze_state_down ) prob += epsilon/5;

        // intended action
        switch(action.get_derived<action_t>()->get_action()) {
        case action_t::ACTION::STAY:
            if(maze_state_to==effective_maze_state_stay ) prob += 1-epsilon;
            break;
        case action_t::ACTION::LEFT:
            if(maze_state_to==effective_maze_state_left ) prob += 1-epsilon;
            break;
        case action_t::ACTION::RIGHT:
            if(maze_state_to==effective_maze_state_right) prob += 1-epsilon;
            break;
        case action_t::ACTION::UP:
            if(maze_state_to==effective_maze_state_up   ) prob += 1-epsilon;
            break;
        case action_t::ACTION::DOWN:
            if(maze_state_to==effective_maze_state_down ) prob += 1-epsilon;
            break;
        default:
            DEBUG_ERROR("Invalid action (" << action << ")");
            return 0;
        }

        if(prob==0) {
            return 0;
        }
    }

    // calculate accumulated reward
    reward_t::value_t accumulated_reward = 0;
    if(reward_vector!=nullptr) {
        reward_vector->assign(rewards.size(),make_pair(0,0));
    }
    for(int r_idx=0; r_idx<(int)rewards.size(); ++r_idx) {
        maze_reward_t r = rewards[r_idx];
        observation_t activate_state = current_observation.new_observation(r[REWARD_ACTIVATION_STATE]);
        observation_t receive_state = current_observation.new_observation(r[REWARD_RECEIVE_STATE]);
        idx_t delay = r[REWARD_TIME_DELAY];
        REWARD_ACTIVATION_TYPE rat = (REWARD_ACTIVATION_TYPE)r[REWARD_ACTIVATION];

        // check if reward could be received
        bool receive_reward = false;
        if(maze_state_to==receive_state) {
            receive_reward = true;
        }

        // check reward only if it can be received or activation type is with
        // punishment for failure
        if(receive_reward || rat==EACH_TIME_PUNISH_FAILURE || rat==ON_RELEASE_PUNISH_FAILURE) {
            idx_t steps_in_past = 1;
            bool reward_invalidated = false;
            for( const_instanceIt_t insIt = instance_from->const_it(); insIt!=INVALID && steps_in_past<=abs(delay); --insIt, ++steps_in_past) {

                // check if delay matches
                bool delay_matches = false;
                if(delay<0 || steps_in_past==delay) {
                    // exact match for positve delay, less-equal match for negative
                    delay_matches = true;
                }

                // check if agent was on activation state
                bool activation_state = false;
                if(insIt->observation==activate_state) {
                    activation_state = true;
                }

                // handle different types of rewards
                switch(rat) {
                case EACH_TIME_NO_PUNISH:
                    if(activation_state && receive_reward && delay_matches ) {
                        // receive reward whenever everything matches
                        accumulated_reward += r[REWARD_VALUE];
                        if(reward_vector!=nullptr) {
                            ++((*reward_vector)[r_idx].first);
                        }
                    }
                    break;
                case ON_RELEASE_NO_PUNISH:
                    if(activation_state && receive_reward && delay_matches && !reward_invalidated) {
                        // successfully receive reward
                        accumulated_reward += r[REWARD_VALUE];
                        if(reward_vector!=nullptr) {
                            ++((*reward_vector)[r_idx].first);
                        }
                    }
                    if(activation_state) {
                        // agent passed activation state and invalidated later
                        // rewards (even if it received this one)
                        reward_invalidated = true;
                    }
                    break;
                case EACH_TIME_PUNISH_FAILURE:
                    if(activation_state && delay_matches ) {
                        // receive reward whenever everything matches, punish if
                        // not on receive-state
                        if(receive_reward) {
                            accumulated_reward += r[REWARD_VALUE];
                            if(reward_vector!=nullptr) {
                                ++((*reward_vector)[r_idx].first);
                            }
                        } else {
                            accumulated_reward -= r[REWARD_VALUE];
                            if(reward_vector!=nullptr) {
                                ++((*reward_vector)[r_idx].second);
                            }
                        }
                    }
                    break;
                case ON_RELEASE_PUNISH_FAILURE:
                    if(activation_state && delay_matches && !reward_invalidated) {
                        // receive reward when everything matches and reward was
                        // not invalidated, punish if not on receive-state
                        if(receive_reward) {
                            accumulated_reward += r[REWARD_VALUE];
                            if(reward_vector!=nullptr) {
                                ++((*reward_vector)[r_idx].first);
                            }
                        } else {
                            accumulated_reward -= r[REWARD_VALUE];
                            if(reward_vector!=nullptr) {
                                ++((*reward_vector)[r_idx].second);
                            }
                        }
                    }
                    if(activation_state) {
                        // agent passed activation state and invalidated later
                        // rewards (even if it received this one or got punished
                        // for missing it)
                        reward_invalidated = true;
                    }
                    break;
                default:
                    DEBUG_DEAD_LINE;
                }

            }
        }
    }

    // check for matching reward
    if(reward->get_value()!=accumulated_reward) {
        return 0;
    }

    return prob;
}

void Maze::get_features(std::vector<f_ptr_t> & basis_features, FeatureLearner::LEARNER_TYPE type) const {

    // clear first
    basis_features.clear();

    // add features
    for(int k_idx = 0; k_idx>=-k; --k_idx) {
        if((type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_ONLY && k_idx<0) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION)) {
            // actions
            for(action_ptr_t action : action_space) {
                f_ptr_t action_feature = ActionFeature::create(action,k_idx);
                DEBUG_OUT(2,"Adding feature: " << *action_feature);
                basis_features.push_back(action_feature);
            }
        }
        if((type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_ONLY && k_idx<0) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION && k_idx<0)) {
            // observations
            for(observation_ptr_t observation : observation_space) {
                f_ptr_t observation_feature = ObservationFeature::create(observation,k_idx);
                DEBUG_OUT(2,"Adding feature: " << *observation_feature);
                basis_features.push_back(observation_feature);
            }
        }
        if((type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE && k_idx==0) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_ONLY && k_idx<0) ||
           (type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION && k_idx<0)) {
            // reward
            for(reward_ptr_t reward : reward_space) {
                f_ptr_t reward_feature = RewardFeature::create(reward,k_idx);
                DEBUG_OUT(2,"Adding feature: " << *reward_feature);
                basis_features.push_back(reward_feature);
            }
        }
    }

    // if(type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE) {
    //     // relative observation features
    //     f_ptr_t relative_observation_feature;
    //     relative_observation_feature = RelativeObservationFeature::create(1,0,-1,0);
    //     basis_features.push_back(relative_observation_feature);
    //     relative_observation_feature = RelativeObservationFeature::create(0,1,-1,0);
    //     basis_features.push_back(relative_observation_feature);
    //     relative_observation_feature = RelativeObservationFeature::create(-1,0,-1,0);
    //     basis_features.push_back(relative_observation_feature);
    //     relative_observation_feature = RelativeObservationFeature::create(0,-1,-1,0);
    //     basis_features.push_back(relative_observation_feature);
    //     relative_observation_feature = RelativeObservationFeature::create(0,0,-1,0);
    //     basis_features.push_back(relative_observation_feature);
    // }
    if(type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION) {
        // also add a unit feature
        f_ptr_t const_feature = ConstFeature::create(1);
        DEBUG_OUT(2,"Adding feature: " << *const_feature);
        basis_features.push_back(const_feature);
    }
}

void Maze::print_reward_activation_on_random_walk(const int& walk_length) {
    // collect data
    vector<pair<long,long> > reward_vector(rewards.size(),make_pair(0,0));
    for(int i=0; i<walk_length; ++i) {
        action_ptr_t action(new MazeAction(action_t::random_action()));
        vector<pair<int,int> > tmp_reward_vector;
        perform_transition(action,&tmp_reward_vector);
        for(int r_idx=0; r_idx<(int)reward_vector.size(); ++r_idx) {
            reward_vector[r_idx].first += tmp_reward_vector[r_idx].first;
            reward_vector[r_idx].second += tmp_reward_vector[r_idx].second;
        }
    }

    // print relativ counts
    DEBUG_OUT(0,"Relative frequencies for reward activation on a length " << walk_length << " random walk");
    for(int r_idx=0; r_idx<(int)reward_vector.size(); ++r_idx) {
        DEBUG_OUT(0,"    Reward " << r_idx <<
                  " (" << current_observation.new_observation(rewards[r_idx][REWARD_ACTIVATION_STATE]) << "," << current_observation.new_observation(rewards[r_idx][REWARD_RECEIVE_STATE]) <<
                  ")	p+ = " << (double)reward_vector[r_idx].first/walk_length <<
                  "	p- = " << (double)reward_vector[r_idx].second/walk_length
            );
    }
}

void Maze::set_epsilon(const double& e) {
    epsilon = e;
}

void Maze::set_current_observation(const observation_ptr_t& observation) {
    typedef std::shared_ptr<const observation_t> o_ptr_t;
    o_ptr_t o = observation.get_derived<observation_t>(false);
    if(o==o_ptr_t()) {
        DEBUG_ERROR("Cast failed. Please provide an observation of correct type");
    } else {
        current_observation = *o;
        delete current_instance;
        current_instance = instance_t::create(action_space, observation_space, reward_space);
        for(idx_t k_idx=0; k_idx<k; ++k_idx) {
            current_instance = current_instance->append_instance(action_space, observation_space, reward_space);
        }
        DEBUG_OUT(1,"Set current state to " << current_observation);
    }
}

const vector<QString> Maze::get_maze_list() {
    vector<QString> return_list;
    for(maze_t maze : maze_list) {
        return_list.push_back(maze.name);
    }
    return return_list;
}

string Maze::get_rewards() {
    stringstream ss;
    for(int r_idx=0; r_idx<(int)rewards.size(); ++r_idx) {
        ss << "Reward " << r_idx << endl;
        ss << "    ACTIVATION_STATE : " << current_observation.new_observation(rewards[r_idx][REWARD_ACTIVATION_STATE]) << endl;
        ss << "    RECEIVE_STATE    : " << current_observation.new_observation(rewards[r_idx][REWARD_RECEIVE_STATE]) << endl;
        ss << "    TIME_DELAY       : " << (int)rewards[r_idx][REWARD_TIME_DELAY] << endl;
        ss << "    reward           : " << rewards[r_idx][REWARD_VALUE] << endl;
        ss << "    activation       : " << reward_activation_type_str((REWARD_ACTIVATION_TYPE)rewards[r_idx][REWARD_ACTIVATION]) << endl;
    }
    return ss.str();
}

string Maze::get_walls() {
    stringstream ss;
    for(int w_idx=0; w_idx<(int)walls.size(); ++w_idx) {
        ss << "Wall " << w_idx << ": (" << walls[w_idx][0] << "|" << walls[w_idx][1] << ")" << endl;
    }
    return ss.str();
}

string Maze::get_doors() {
    stringstream ss;
    for(door_t d : doors) {
        ss << "Door (" << get<DOOR_STATE_FROM>(d) << "/" << get<DOOR_STATE_TO>(d) << ")" << endl;
        ss << "    key : " << get<DOOR_KEY_STATE>(d) << " : ";
        switch(get<DOOR_KEY>(d)) {
        case PASS_BUTTON:
            ss << "PASS_BUTTON";
            break;
        case STAY_BUTTON:
            ss << "STAY_BUTTON";
            break;
        case UP_BUTTON:
            ss << "UP_BUTTON";
            break;
        case DOWN_BUTTON:
            ss << "DOWN_BUTTON";
            break;
        case LEFT_BUTTON:
            ss << "LEFT_BUTTON";
            break;
        case RIGHT_BUTTON:
            ss << "RIGHT_BUTTON";
            break;
        default:
            DEBUG_DEAD_LINE;
        }
        ss << endl;
        ss << "    dt : " << get<DOOR_TIME_DELAY>(d) << endl;
    }
    return ss.str();
}

void Maze::print_transition(action_ptr_t& a, observation_ptr_t& o, reward_ptr_t& r) const {
    // check
    if(current_maze.name!="Default" && current_maze.name!="Minimal") {
        DEBUG_ERROR("UTF-8 printing of transitions is only supported for maze 'Default'");
        return;
    }

    // get string for button and door
    const char * button;
    const char * door;
    if(current_maze.name=="Default") {
        button = "◗";
        door = "╲";
    } else {
        button = " ";
        door = "┃";
    }
    // get action string
    const char * as;
    if(a==MazeAction("up")) {
        as = "↑";
    } else if(a==MazeAction("down")) {
        as = "↓";
    } else if(a==MazeAction("right")) {
        as = "→";
    } else if(a==MazeAction("left")) {
        as = "←";
    } else if(a==MazeAction("stay")) {
        as = "●";
    } else {
        DEBUG_ERROR("unexpected action (" << a << ")");
    }
    // get reward colorization
    QString rc, rs = ColorOutput::reset_all().c_str();
    if(r->get_value()==0) {
        rc = "";
    } else if(r->get_value()==1) {
        rc = ColorOutput::fg_red().c_str();
    } else {
        DEBUG_ERROR("unexpected reward " << r);
    }
    // print the thing
    if(o==MazeObservation(2,2,0,0)) {
        std::cout << "┏━━━┳━━━┓" << std::endl;
        std::cout << "┃"<<button<<""<<rc<<"●"<<rs<<" "<<door<<"   ┃" << std::endl;
        std::cout << "┣━━━╋━━━┫"<<as << std::endl;
        std::cout << "┃   ┃   ┃" << std::endl;
        std::cout << "┗━━━┻━━━┛" << std::endl;
    } else if(o==MazeObservation(2,2,0,1)) {
        std::cout << "┏━━━┳━━━┓" << std::endl;
        std::cout << "┃"<<button<<"  "<<door<<"   ┃" << std::endl;
        std::cout << "┣━━━╋━━━┫"<<as << std::endl;
        std::cout << "┃ "<<rc<<"●"<<rs<<" ┃   ┃" << std::endl;
        std::cout << "┗━━━┻━━━┛" << std::endl;
    } else if(o==MazeObservation(2,2,1,0)) {
        std::cout << "┏━━━┳━━━┓" << std::endl;
        std::cout << "┃"<<button<<"  "<<door<<" "<<rc<<"●"<<rs<<" ┃" << std::endl;
        std::cout << "┣━━━╋━━━┫"<<as << std::endl;
        std::cout << "┃   ┃   ┃" << std::endl;
        std::cout << "┗━━━┻━━━┛" << std::endl;
    } else if(o==MazeObservation(2,2,1,1)) {
        std::cout << "┏━━━┳━━━┓" << std::endl;
        std::cout << "┃"<<button<<"  "<<door<<"   ┃" << std::endl;
        std::cout << "┣━━━╋━━━┫"<<as << std::endl;
        std::cout << "┃   ┃ "<<rc<<"●"<<rs<<" ┃" << std::endl;
        std::cout << "┗━━━┻━━━┛" << std::endl;
    } else {
        DEBUG_ERROR("unexpected observation (" << o << ")");
    }
}

void Maze::frame_maze() {

    QGraphicsScene * scene = view->scene();

    // determine border dimensions
    double border_min_x = DBL_MAX;
    double border_min_y = DBL_MAX;
    double border_max_x = -DBL_MAX;
    double border_max_y = -DBL_MAX;
    for(observation_ptr_t o : observation_space) {
        border_min_x = min(border_min_x,(double)o.get_derived<observation_t>()->get_x_pos());
        border_min_y = min(border_min_y,(double)o.get_derived<observation_t>()->get_y_pos());
        border_max_x = max(border_max_x,(double)o.get_derived<observation_t>()->get_x_pos());
        border_max_y = max(border_max_y,(double)o.get_derived<observation_t>()->get_y_pos());
    }
    double border_x = border_min_x - state_size/2 - border_margin;
    double border_y = border_min_y - state_size/2 - border_margin;
    double border_width  = border_max_x - border_min_x + state_size + 2*border_margin;
    double border_height = border_max_y - border_min_y + state_size + 2*border_margin;

    // determine reward range
    double min_reward = DBL_MAX;
    double max_reward = -DBL_MAX;
    for(reward_ptr_t r : reward_space) {
        min_reward = min(min_reward,r->get_value());
        max_reward = max(max_reward,r->get_value());
    }
    double reward_magnitude = max_reward - min_reward;
    if(reward_magnitude==0) {
        reward_magnitude = 1;
    }

    // create frames for different reward values
    for(reward_ptr_t r : reward_space) {
        double intensity = r->get_value()/reward_magnitude;
        bool positive = intensity>=0;
        intensity = fabs(intensity);
        if(intensity!=0) {
            // to clearly distinguish reward from non-reward
            intensity = intensity/2 + 0.5;
        }
        intensity *= 255;
        QPen border_pen(QColor(positive?0:intensity,positive?intensity:0,0), 0.04, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        borders.push_back( scene->addRect( border_x, border_y, border_width, border_height, border_pen, QBrush(QColor(255,255,255)) ) );
        if(intensity!=0) {
            borders.back()->setVisible(false);
        }
    }
}

void Maze::render_state(observation_ptr_t observation) {

    // get corret observation type
    observation_t o = *(observation.get_derived<observation_t>());

    // get scene
    QGraphicsScene * scene = view->scene();

    // create state
    QPen state_pen(QColor(0,0,0), 0.02, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QGraphicsRectItem * rect = scene->addRect( o.get_x_pos()-state_size/2,
                                               o.get_y_pos()-state_size/2,
                                               state_size,
                                               state_size,
                                               state_pen,
                                               QBrush(QColor(255,255,255))
        );
    state_rects.push_back(rect);
}

void Maze::render_wall(wall_t w) {

    QGraphicsScene * scene = view->scene();

    QPen wall_pen(QColor(50,50,50), 0.02, Qt::SolidLine, Qt::RoundCap);
    QBrush wall_brush(QColor(50,50,50));

    observation_t maze_state_1 = current_observation.new_observation(w[0]);
    observation_t maze_state_2 = current_observation.new_observation(w[1]);
    idx_t x_1 = maze_state_1.get_x_pos();
    idx_t y_1 = maze_state_1.get_y_pos();
    idx_t x_2 = maze_state_2.get_x_pos();
    idx_t y_2 = maze_state_2.get_y_pos();
    if( abs(x_1-x_2)==1 && abs(y_1-y_2)==0 ) {
        idx_t x_min = min<idx_t>(x_1,x_2);
        idx_t y = y_1;
        scene->addRect( x_min+0.5-wall_width/2, y-0.5, wall_width, 1, wall_pen, wall_brush );
        scene->addEllipse( x_min+0.5-wall_width/2, y-0.5-wall_width/2, wall_width, wall_width, wall_pen, wall_brush );
        scene->addEllipse( x_min+0.5-wall_width/2, y+0.5-wall_width/2, wall_width, wall_width, wall_pen, wall_brush );
    } else if( abs(x_1-x_2)==0 && abs(y_1-y_2)==1 ) {
        idx_t x = x_1;
        idx_t y_min = min<idx_t>(y_1,y_2);
        scene->addRect( x-0.5, y_min+0.5-wall_width/2, 1, wall_width, wall_pen, wall_brush );
        scene->addEllipse( x-0.5-wall_width/2, y_min+0.5-wall_width/2, wall_width, wall_width, wall_pen, wall_brush );
        scene->addEllipse( x+0.5-wall_width/2, y_min+0.5-wall_width/2, wall_width, wall_width, wall_pen, wall_brush );
    } else {
        DEBUG_ERROR("No wall possible between (" <<
                  x_1 << "," << y_1 << ") and (" <<
                  x_2 << "," << y_2 << ")" );
    }
}

void Maze::render_door(door_t dt) {

    QGraphicsScene * scene = view->scene();

    observation_t s1 = get<DOOR_STATE_FROM>(dt);
    observation_t s2 = get<DOOR_STATE_TO>(dt);
    observation_t s3 = get<DOOR_KEY_STATE>(dt);
    KEY_TYPE kt =  get<DOOR_KEY>(dt);
    idx_t delay =  get<DOOR_TIME_DELAY>(dt);
    color_t c = get<DOOR_COLOR>(dt);

    QColor door_color(255*get<COLOR_R>(c),255*get<COLOR_G>(c),255*get<COLOR_B>(c));
    QPen door_pen(door_color, 0.05, Qt::SolidLine, Qt::RoundCap);
    QBrush door_brush(door_color);

    // draw door
    idx_t x1 = s1.get_x_pos();
    idx_t y1 = s1.get_y_pos();
    idx_t x2 = s2.get_x_pos();
    idx_t y2 = s2.get_y_pos();
    if( abs(x1-x2)==1 && abs(y1-y2)==0 ) {
        idx_t x_min = min<idx_t>(x1,x2);
        idx_t y = y1;
        scene->addLine( x_min+0.5, y-0.3, x_min+0.7, y+0.3, door_pen );
    } else if( abs(x1-x2)==0 && abs(y1-y2)==1 ) {
        idx_t x = x1;
        idx_t y_min = min<idx_t>(y1,y2);
        scene->addLine( x+0.3, y_min+0.5, x-0.3, y_min+0.7, door_pen );
    } else {
        DEBUG_ERROR("No door possible between (" <<
                  x1 << "," << y1 << ") and (" <<
                  x2 << "," << y2 << ")" );
    }

    // draw key
    idx_t x3 = s3.get_x_pos();
    idx_t y3 = s3.get_y_pos();
    double text_x, text_y;
    switch(kt) {
    case PASS_BUTTON:
        scene->addEllipse( x3-0.1, y3-0.1, 0.2, 0.2, door_pen );
        text_x = x3;
        text_y = y3;
        break;
    case STAY_BUTTON:
        scene->addEllipse( x3-0.1, y3-0.1, 0.2, 0.2, door_pen, door_brush );
        text_x = x3;
        text_y = y3;
        break;
    case UP_BUTTON:
        scene->addEllipse( x3-0.1, y3-state_size/2, 0.2, 0.05, door_pen, door_brush );
        text_x = x3;
        text_y = y3-state_size/4;
        break;
    case DOWN_BUTTON:
        scene->addEllipse( x3-0.1, y3+state_size/2-0.05, 0.2, 0.05, door_pen, door_brush );
        text_x = x3;
        text_y = y3+state_size/4;
        break;
    case LEFT_BUTTON:
        scene->addEllipse( x3-state_size/2, y3-0.1, 0.05, 0.2, door_pen, door_brush );
        text_x = x3-state_size/4;
        text_y = y3;
        break;
    case RIGHT_BUTTON:
        scene->addEllipse( x3+state_size/2-0.05, y3-0.1, 0.05, 0.2, door_pen, door_brush );
        text_x = x3+state_size/4;
        text_y = y3;
        break;
    }

    // add text
    if(draw_text) {
        QGraphicsTextItem * txt = scene->addText(
            QString("t=%1").arg(QString::number(delay)),
            text_font
            );
        QRectF box = txt->boundingRect();
        txt->setPos(
            text_x-text_scale*box.width()/2,
            text_y-text_scale*box.height()/2
            );
        txt->setScale(text_scale);
        txt->setDefaultTextColor(door_color);
    }
}

void Maze::render_reward(maze_reward_t r) {

    QGraphicsScene * scene = view->scene();

    observation_t maze_state_1 = current_observation.new_observation((int)r[REWARD_ACTIVATION_STATE]);
    observation_t maze_state_2 = current_observation.new_observation((int)r[REWARD_RECEIVE_STATE]);
    double x_start   = maze_state_1.get_x_pos();
    double y_start   = maze_state_1.get_y_pos();
    double x_end     = maze_state_2.get_x_pos();
    double y_end     = maze_state_2.get_y_pos();
    double x_shift   = -(y_end-y_start)/5;
    double y_shift   = (x_end-x_start)/5;
    double x_control = (x_start+x_end)/2+x_shift;
    double y_control = (y_start+y_end)/2+y_shift;
    QColor color( (int)r[REWARD_R], (int)r[REWARD_G], (int)r[REWARD_B]);
    QPen   arc_pen( color,0.02,Qt::SolidLine,Qt::RoundCap );
    QPen   marker_pen( color,0.02,Qt::SolidLine,Qt::RoundCap,Qt::MiterJoin );
    QBrush brush( color );
    QBrush no_brush( Qt::NoBrush );

    // arc
    QPainterPath arc_path;
    arc_path.moveTo(x_start, y_start);
    arc_path.quadTo( x_control, y_control, x_end, y_end );
    scene->addPath(arc_path,arc_pen,QBrush(QColor(0,0,0,0)));

    // start
    switch((REWARD_ACTIVATION_TYPE)r[REWARD_ACTIVATION]) {
    case EACH_TIME_NO_PUNISH:
        scene->addEllipse(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            brush
            );
        break;
    case EACH_TIME_PUNISH_FAILURE:
        scene->addRect(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            brush
            );
        break;
    case ON_RELEASE_NO_PUNISH:
        scene->addEllipse(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            no_brush
            );
        break;
    case ON_RELEASE_PUNISH_FAILURE:
        scene->addRect(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            no_brush
            );
        break;
    default:
        DEBUG_DEAD_LINE;
    }

    //========================//
    // drawing the end marker //
    //========================//
    // normal vector in end direction
    double end_vector_x = x_end-x_control;
    double end_vector_y = y_end-y_control;
    double end_vector_norm = sqrt( pow(end_vector_x,2) + pow(end_vector_y,2) );
    end_vector_x /= end_vector_norm;
    end_vector_y /= end_vector_norm;
    // normal vector perpendicular to end direction
    double end_cross_vector_x = +end_vector_y;
    double end_cross_vector_y = -end_vector_x;
    // draw arrow
    QPainterPath end_path;
    end_path.moveTo(x_end, y_end); // point
    end_path.lineTo(
        x_end-reward_end_size*(end_vector_x+end_cross_vector_x*reward_end_ratio/2),
        y_end-reward_end_size*(end_vector_y+end_cross_vector_y*reward_end_ratio/2)
        ); // first wing
    end_path.lineTo(
        x_end-reward_end_size*(end_vector_x*(1-reward_end_notch)),
        y_end-reward_end_size*(end_vector_y*(1-reward_end_notch))
        ); // notch
    end_path.lineTo(
        x_end-reward_end_size*(end_vector_x-end_cross_vector_x*reward_end_ratio/2),
        y_end-reward_end_size*(end_vector_y-end_cross_vector_y*reward_end_ratio/2)
        ); // second wing
    end_path.lineTo(x_end, y_end); // close at point
    scene->addPath(end_path,marker_pen,QBrush(color));

    // text
    if(draw_text) {
        double mid_point_x = x_start + (x_end - x_start)/2;
        double mid_point_y = y_start + (y_end - y_start)/2;
        idx_t time_delay = (idx_t)r[REWARD_TIME_DELAY];
        reward_t::value_t reward = r[REWARD_VALUE];
        QGraphicsTextItem * txt = scene->addText(
            QString("t=%1,r=%2").arg(QString::number(time_delay)).arg(QString::number(reward)),
            text_font
            );
        QRectF box = txt->boundingRect();
        txt->setPos(
            x_control+(mid_point_x-x_control)*text_center-text_scale*box.width()/2,
            y_control+(mid_point_y-y_control)*text_center-text_scale*box.height()/2
            );
        txt->setScale(text_scale);
        txt->setDefaultTextColor(color);
    }
}

const char* Maze::reward_activation_type_str(REWARD_ACTIVATION_TYPE ra) {
    switch(ra) {
    case EACH_TIME_NO_PUNISH:
        return "EACH_TIME_NO_PUNISH";
    case ON_RELEASE_NO_PUNISH:
        return "ON_RELEASE_NO_PUNISH";
    case EACH_TIME_PUNISH_FAILURE:
        return "EACH_TIME_PUNISH_FAILURE";
    case ON_RELEASE_PUNISH_FAILURE:
        return "ON_RELEASE_PUNISH_FAILURE";
    default:
        DEBUG_DEAD_LINE;
        return "Error!";
    }
}

bool Maze::check_maze_definition() const {

    typedef std::vector<idx_t> wall_t;

    bool all_walls_ok = true;
    for(wall_t wall : walls) {
        // check if both states exist
        bool this_wall_ok_0 = false;
        bool this_wall_ok_1 = false;
        for(observation_ptr_t o : observation_space) {
            if(o.get_derived<MazeObservation>()->get_index()==wall[0]) {
                this_wall_ok_0 = true;
            }
            if(o.get_derived<MazeObservation>()->get_index()==wall[1]) {
                this_wall_ok_1 = true;
            }
            if(this_wall_ok_0 && this_wall_ok_1) {
                break;
            }
        }
        // check if states are next to each other
        MazeObservation o0 = observation_space.get_derived<MazeObservation>()->new_observation(wall[0]);
        MazeObservation o1 = observation_space.get_derived<MazeObservation>()->new_observation(wall[1]);
        bool next_to_each_other = ( (abs(o0.get_x_pos()-o1.get_x_pos())==0 && abs(o0.get_y_pos()-o1.get_y_pos())==1) ||
                                    (abs(o0.get_x_pos()-o1.get_x_pos())==1 && abs(o0.get_y_pos()-o1.get_y_pos())==0) );
        // process result
        if(!this_wall_ok_0) {
            DEBUG_WARNING("incorrect wall: " << o0 << " is not within observation space");
        }
        if(!this_wall_ok_1) {
            DEBUG_WARNING("incorrect wall: " << o1 << " is not within observation space");
        }
        if(!next_to_each_other) {
            DEBUG_WARNING("incorrect wall: " << o0 << " and " << o1 << " are NOT next to each other");
        }
        if(!this_wall_ok_0 || !this_wall_ok_1 || !next_to_each_other) {
            all_walls_ok = false;
        }
    }

    bool all_rewards_ok = true;
    for(maze_reward_t reward : rewards) {
        // check if activation and receive state exist
        bool this_reward_ok_activation = false;
        bool this_reward_ok_receive = false;
        for(observation_ptr_t o : observation_space) {
            if(o.get_derived<MazeObservation>()->get_index()==reward[REWARD_ACTIVATION_STATE]) {
                this_reward_ok_activation = true;
            }
            if(o.get_derived<MazeObservation>()->get_index()==reward[REWARD_RECEIVE_STATE]) {
                this_reward_ok_receive = true;
            }
            if(this_reward_ok_activation && this_reward_ok_receive) {
                break;
            }
        }
        // check if reward value (and opposite value for rewards with
        // punishment) exists
        bool reward_value_ok = false;
        bool punish_value_ok = false;
        double reward_value = reward[REWARD_VALUE];
        for(reward_ptr_t r : reward_space) {
            if(r->get_value()==reward_value) {
                reward_value_ok = true;
            }
            if(reward[REWARD_ACTIVATION]==EACH_TIME_PUNISH_FAILURE ||
               reward[REWARD_ACTIVATION]==ON_RELEASE_PUNISH_FAILURE) {
                if(r->get_value()==(-1*reward_value)) {
                    punish_value_ok = true;
                }
            } else {
                punish_value_ok = reward_value_ok;
            }
            if(reward_value_ok && punish_value_ok) {
                break;
            }
        }
        // process result
        if(!this_reward_ok_activation) {
            DEBUG_WARNING("incorrect reward: index " << reward[REWARD_ACTIVATION_STATE] << " is not within observation space");
        }
        if(!this_reward_ok_receive) {
            DEBUG_WARNING("incorrect reward: index" << reward[REWARD_RECEIVE_STATE] << " is not within observation space");
        }
        if(!reward_value_ok) {
            DEBUG_WARNING("incorrect reward: value " << reward[REWARD_VALUE] << " is not within reward space");
        }
        if(!punish_value_ok) {
            DEBUG_WARNING("incorrect reward: punish value " << -reward[REWARD_VALUE] << " is not within reward space");
        }
        if(!this_reward_ok_activation ||
           !this_reward_ok_receive ||
           !reward_value_ok ||
           !punish_value_ok) {
            all_rewards_ok = false;
        }
    }

    bool all_doors_ok = true;
    for(door_t door : doors) {
        // check if both states exist
        bool this_door_ok_FROM = false;
        bool this_door_ok_TO = false;
        bool this_door_ok_KEY = false;
        for(observation_ptr_t o : observation_space) {
            if(o==get<DOOR_STATE_FROM>(door)) {
                this_door_ok_FROM = true;
            }
            if(o==get<DOOR_STATE_TO>(door)) {
                this_door_ok_TO = true;
            }
            if(o==get<DOOR_KEY_STATE>(door)) {
                this_door_ok_KEY = true;
            }
            if(this_door_ok_FROM && this_door_ok_TO && this_door_ok_KEY) {
                break;
            }
        }
        // check if states are next to each other
        MazeObservation o_FROM = get<DOOR_STATE_FROM>(door);
        MazeObservation o_TO = get<DOOR_STATE_TO>(door);
        MazeObservation o_KEY = get<DOOR_KEY_STATE>(door); // for warnings only
        bool next_to_each_other = ( (abs(o_FROM.get_x_pos()-o_TO.get_x_pos())==0 && abs(o_FROM.get_y_pos()-o_TO.get_y_pos())==1) ||
                                    (abs(o_FROM.get_x_pos()-o_TO.get_x_pos())==1 && abs(o_FROM.get_y_pos()-o_TO.get_y_pos())==0) );
        // process result
        if(!this_door_ok_FROM) {
            DEBUG_WARNING("incorrect door: " << o_FROM << " is not within observation space");
        }
        if(!this_door_ok_TO) {
            DEBUG_WARNING("incorrect door: " << o_TO << " is not within observation space");
        }
        if(!this_door_ok_KEY) {
            DEBUG_WARNING("incorrect door: " << o_KEY << " is not within observation space");
        }
        if(!next_to_each_other) {
            DEBUG_WARNING("incorrect door: " << o_FROM << " and " << o_TO << " are NOT next to each other");
        }
        if(!this_door_ok_FROM || !this_door_ok_TO || !this_door_ok_KEY || !next_to_each_other) {
            all_doors_ok = false;
        }
    }

    return all_walls_ok && all_rewards_ok && all_doors_ok;
}
