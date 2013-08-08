#include "Maze.h"
#include "util.h"

#define DEBUG_LEVEL 0
#include "debug.h"

using std::tuple;
using std::get;
using std::vector;
using std::get;

using util::min;
using util::max;
using util::INVALID;

const double Maze::state_size = 0.9;
const double Maze::wall_width = 0.08;
const double Maze::reward_start_size = 0.15;
const double Maze::reward_end_size = 0.2;
const double Maze::reward_end_ratio = 0.5;
const double Maze::text_scale = 0.01;
const double Maze::text_center = 0.3;

const vector<Maze::wall_t> Maze::walls = {

    /* 2x2 Maze *
    { 0, 1}
    /**/

    /* 3x3 Maze *
    { 0, 1},
    { 0, 3},
    { 2, 1},
    { 2, 5},
    { 6, 3},
    { 6, 7},
    { 8, 7},
    { 8, 5}
    /**/

    /* 4x4 Maze *
    { 6, 7},
    {12,13},
    { 4, 5},
    { 6,10},
    {10,11}
    /**/

    /* 10x10 Maze */
    {  2,  3},
    { 12, 13},
    { 22, 23},
    { 32, 33},
    { 30, 40},
    { 31, 41},
    { 33, 34},
    { 43, 44},
    { 53, 54},
    { 63, 64},
    { 73, 74},
    { 83, 84},
    { 61, 71},
    { 62, 72},
    { 63, 73},
    { 44, 45},
    { 54, 55},
    { 74, 75},
    { 84, 85},
    { 75, 85},
    { 76, 86},
    { 77, 87},
    { 23, 24},
    { 25, 35},
    { 26, 36},
    { 27, 37},
    { 28, 38},
    {  6,  7},
    { 16, 17},
    { 78, 88},
    { 59, 69},
    { 58, 68},
    { 57, 67},
    { 56, 66}
    /**/
};

const vector<Maze::maze_reward_t> Maze::rewards = {
    /* 2x2 Maze *
    { 0, 3, 4, 5, ON_RELEASE, 200,   0,   0},
    { 3, 0, 4, 5, ON_RELEASE, 200, 200,   0},
    { 0, 1, 1, 1, ON_RELEASE,   0, 200,   0},
    { 3, 2, 1, 1, ON_RELEASE,   0,   0, 200}
    /**/

    /* 2x2 Maze *
    { 0, 3, 2, 1, EACH_TIME, 200,   0,   0}
    /**/

    /* 3x3 Maze *
    { 3, 5, 4, 8, ON_RELEASE,   0, 200,   0},
    { 5, 3, 6, 8, ON_RELEASE,   0, 200, 200},
    { 4, 1, 1, 1, ON_RELEASE, 200, 200,   0},
    { 4, 7, 3, 3, ON_RELEASE, 200,   0,   0}
    /**/

    /* 4x4 Maze *
    {  4,  2,  3, 1, ON_RELEASE, 200,   0,   0},
    {  6,  7,  3, 1, ON_RELEASE, 200, 200,   0},
    { 11, 14,  2, 1, ON_RELEASE,   0, 200,   0},
    { 13,  8,  2, 1, ON_RELEASE,   0, 200, 200},
    {  8,  1,  3, 1, ON_RELEASE,   0,   0, 200}
    //{  8,  4,  4, 1, ON_RELEASE, 200,   0, 200}
    /**/

    /* 10x10 Maze */
    {  0, 21,  3, 1, ON_RELEASE, 200,   0,   0},
    { 30, 43,  4, 1, ON_RELEASE,   0, 200,   0},
    { 43, 62,  3, 1, ON_RELEASE, 200,   0,   0},
    { 61, 92, -4, 1, ON_RELEASE,   0, 200,   0},
    { 94, 86,  3, 1, ON_RELEASE, 200,   0,   0},
    { 87, 66,  3, 1, ON_RELEASE, 200,   0,   0},
    { 67, 59,  3, 1, ON_RELEASE, 200,   0,   0},
    { 48, 56,  3, 1, ON_RELEASE, 200,   0,   0},
    { 55, 36,  3, 1, ON_RELEASE, 200,   0,   0},
    { 59, 38,  3, 1, ON_RELEASE, 200,   0,   0},
    { 28,  9,  3, 1, ON_RELEASE, 200,   0,   0},
    { 18, 37,  3, 1, ON_RELEASE, 200,   0,   0},
    { 25, 33,  3, 1, ON_RELEASE, 200,   0,   0},
    { 33,  3,  3, 1, ON_RELEASE, 200,   0,   0},
    {  4,  1,  3, 1, ON_RELEASE, 200,   0,   0},
    {  8, 17,  2, 1, ON_RELEASE,   0,   0, 200},
    { 40, 50,  2, 1, ON_RELEASE,   0, 200, 200}, // test
    { 41, 51,  2, 1,  EACH_TIME,   0, 200, 200}, // test
    { 60, 70,  2, 1, ON_RELEASE_PUNISH_FAILURE,   0, 200, 200}, // test
    { 80, 90,  2, 1,  EACH_TIME_PUNISH_FAILURE,   0, 200, 200}  // test
    /**/
};

const vector<Maze::door_t> Maze::doors = {
    /* 10x10 Maze */
    door_t(MazeState(0,3), MazeState(0,4), MazeState(0,4),  PASS_BUTTON, 0, color_t(1.0,0.5,0.0) ),
    door_t(MazeState(1,3), MazeState(1,4), MazeState(1,5),  STAY_BUTTON, 2, color_t(0.0,1.0,0.0) ),
    door_t(MazeState(2,2), MazeState(3,2), MazeState(2,3),    UP_BUTTON, 2, color_t(0.0,1.0,0.0) ),
    door_t(MazeState(3,3), MazeState(4,3), MazeState(3,3),  DOWN_BUTTON, 3, color_t(0.0,0.5,0.5) ),
    door_t(MazeState(4,5), MazeState(5,5), MazeState(4,6),  LEFT_BUTTON,-5, color_t(0.0,0.0,1.0) ),
    door_t(MazeState(6,5), MazeState(6,6), MazeState(4,7), RIGHT_BUTTON, 5, color_t(1.0,0.0,1.0) )
    /**/
};

Maze::Maze(const double& eps):
    current_instance(nullptr),
    view(nullptr),
    epsilon(eps),
    agent(nullptr)
{
    // setting current state
    current_instance = instance_t::create(action_t::STAY, current_state.state_idx(), reward_t(0));
    current_state = MazeState(Config::maze_x_size/2, Config::maze_y_size/2);
    DEBUG_OUT(1,"Current Maze State: " << current_state << " (Index: " << current_state.state_idx() << ")" );
    set_current_state(current_state.state_idx());

    // TODO: Test if all rewards are actually valid!
}


Maze::~Maze() {
    delete agent;
    delete current_instance;
//    delete button;
//    delete smiley;
}


void Maze::render_initialize(QGraphicsView * v) {

    // Set view
    view = v;

    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();
    if(scene==NULL) {
        scene = new QGraphicsScene();
        view->setScene(scene);
    }

    // Put Border Around Maze
    frame_maze();

    // Render States
    state_rects.clear();
    for(auto state : stateIt_t::all) {
        render_state(state);
    }

    // Render Walls
    for( auto w : walls ) {
        render_wall(w);
    }

    // Render Doors
    for( auto d : doors ) {
        render_door(d);
    }

    // Rewards
    for(auto reward : rewards ) {
        render_reward(reward);
    }

    // render agent
    if(!agent) {
        agent = new QGraphicsSvgItem("agent.svg");
        agent->setScale(0.2);
        QSizeF s = agent->boundingRect().size();
        agent->setPos(current_state.x()-s.width()/2, current_state.y()-s.height()/2);
        agent->setElementId("normal");
    }

    scene->addItem(agent);

    rescale_scene(view);
}


void Maze::render_update(const color_vector_t * color) {
    // set agent position and mirror to make 'stay' actions visible
    QSizeF s = agent->boundingRect().size();
    agent->setPos(current_state.x()-agent->scale()*s.width()/2, current_state.y()-agent->scale()*s.height()/2);
    agent->setElementId(agent->elementId()=="normal" ? "mirrored" : "normal");

    // show reward by border color
    int border_idx = 0;
    bool matching_reward = false; // safety check
    for(rewardIt_t rewIt=rewardIt_t::first(); rewIt!=INVALID; ++rewIt, ++border_idx) {
        if(current_instance->reward==rewIt) {
            borders[border_idx]->setVisible(true);
            matching_reward = true;
        } else {
            borders[border_idx]->setVisible(false);
        }
    }

    if(!matching_reward) {
        DEBUG_OUT(0,"Error: Current reward is invalid (" << current_instance->reward << ")");
    }

    // Change State Color
    if(color!=nullptr) {
        idx_t col_idx = 0;
        for( auto rect_ptr : state_rects ) {
            rect_ptr->setBrush( QColor( get<0>((*color)[col_idx])*255, get<1>((*color)[col_idx])*255, get<2>((*color)[col_idx])*255 ) );
            ++col_idx;
        }
    }

    rescale_scene(view);
}


void Maze::perform_transition(const action_t& action) {
    MazeState old_state = current_state; // remember current (old) state

    if(DEBUG_LEVEL>=1) {
        DEBUG_OUT(1,"Current instance: ");
        const_instanceIt_t insIt = current_instance->it();
        for(idx_t k_idx=0; k_idx<(idx_t)Config::k; ++k_idx) {
            DEBUG_OUT(1,"    " << (*insIt) );
            --insIt;
        }
    }

    // perform transition
    probability_t prob_threshold = drand48();
    DEBUG_OUT(2,"Prob threshold = " << prob_threshold);
    probability_t prob_accum = 0;
    bool was_set = false;
    for(stateIt_t state_to=stateIt_t::first(); state_to!=INVALID && !was_set; ++state_to) {
        for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID && !was_set; ++reward) {
            probability_t prob = get_prediction(current_instance, action, state_to, reward);
            DEBUG_OUT(2,"state(" << state_to << "), reward(" << reward << ") --> prob=" << prob);
            prob_accum += prob;
            if(prob_accum>prob_threshold) {
                current_state = MazeState(state_to);
                current_instance = current_instance->append_instance(action, state_to, reward);
                was_set = true;
                DEBUG_OUT(2,"CHOOSE");
            }
        }
    }
    if(!was_set) {
        DEBUG_OUT(0, "Error: Unnormalized probabilities [sum(p)=" << prob_accum << "]--> no transition performed." );
    }

    DEBUG_OUT(1, "(" <<
              old_state.x() << "," <<
              old_state.y() << ") + " <<
              action << " ==> (" <<
              current_state.x() << "," <<
              current_state.y() << ")"
        );
}


void Maze::perform_transition(const action_t& a, state_t& final_state, reward_t& r) {
    perform_transition(a);
    final_state = current_state.state_idx();
    r = current_instance->reward;
}

Maze::probability_t Maze::get_prediction(const instance_t* instance_from, const action_t& action, const state_t& state_to, const reward_t& reward) const {

    // state 'from' and 'to'
    MazeState maze_state_from(instance_from->state);
    MazeState maze_state_to(state_to);
    int x_from = maze_state_from.x();
    int y_from = maze_state_from.y();

    // states that can in principle be reached (ignoring walls and doors)
    MazeState maze_state_stay(                               x_from   ,                                y_from   );
    MazeState maze_state_left( clamp(0,Config::maze_x_size-1,x_from-1),                                y_from   );
    MazeState maze_state_right(clamp(0,Config::maze_x_size-1,x_from+1),                                y_from   );
    MazeState maze_state_up(                                 x_from   , clamp(0,Config::maze_y_size-1, y_from-1));
    MazeState maze_state_down(                               x_from   , clamp(0,Config::maze_y_size-1, y_from+1));

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
    MazeState reachable_states[4] = { maze_state_left, maze_state_right, maze_state_up, maze_state_down };
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
        MazeState s1 = get<0>(d);
        MazeState s2 = get<1>(d);
        MazeState s3 = get<2>(d);
        KEY_TYPE kt =  get<3>(d);
        idx_t delay =  get<4>(d);

        // iterate through reachable state
        for(int idx=0; idx<4; ++idx) {
            MazeState tmp_maze_state_to = reachable_states[idx];

            // check if transition was unblocked already
            if(*unblocked_by_door[idx]) {
                continue;
            }

            // check if the door connects the two states
            if( (tmp_maze_state_to==s1 && maze_state_from==s2) || (tmp_maze_state_to==s2 && maze_state_from==s1) ) {

                // check if key was activated
                idx_t steps_in_past = 0;
                for( const_instanceIt_t insIt = instance_from->const_it(); insIt!=INVALID && steps_in_past<=abs(delay); --insIt, ++steps_in_past) {

                    // check if delay matches
                    if(delay<0 || steps_in_past==delay) { // exact match for positve delay less-equal match for negative

                        // check if key state was visited at that time
                        if(MazeState(insIt->state)==s3) {

                            // check action
                            action_t past_action = action;
                            if(steps_in_past>0) {
                                past_action = (insIt+1)->action;
                            }
                            switch(kt) {
                            case PASS_BUTTON:
                                // action does not matter
                                *unblocked_by_door[idx]=true;
                                break;
                            case STAY_BUTTON:
                                if(past_action==(action_t)action_t::STAY) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case UP_BUTTON:
                                if(past_action==(action_t)action_t::UP) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case DOWN_BUTTON:
                                if(past_action==(action_t)action_t::DOWN) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case LEFT_BUTTON:
                                if(past_action==(action_t)action_t::LEFT) {
                                    *unblocked_by_door[idx]=true;
                                }
                                break;
                            case RIGHT_BUTTON:
                                if(past_action==(action_t)action_t::RIGHT) {
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

    // check for walls (ignore walls when transition was unblocked by a door)
    for(auto w : walls) {
        MazeState s1(w[0]);
        MazeState s2(w[1]);

        // iterate through reachable states
        for(int idx=0; idx<4; ++idx) {
            MazeState tmp_maze_state_to = reachable_states[idx];

            // check if transition was unblocked by door or already blocked by wall
            if(*unblocked_by_door[idx] || *blocked_by_wall[idx]) {
                continue;
            }

            // check if wall is between the two states
            if( (maze_state_from==s1 && tmp_maze_state_to==s2) || (maze_state_from==s2 && tmp_maze_state_to==s1) ) {
                *blocked_by_wall[idx] = true;
            }
        }
    }

    // determine effective states (by considering blocking/unblocking)
    MazeState effective_maze_state_stay  = maze_state_stay;
    MazeState effective_maze_state_left  = ( left_unblocked_by_door || ! left_blocked_by_wall) ? maze_state_left  : maze_state_stay;
    MazeState effective_maze_state_right = (right_unblocked_by_door || !right_blocked_by_wall) ? maze_state_right : maze_state_stay;
    MazeState effective_maze_state_up    = (   up_unblocked_by_door || !   up_blocked_by_wall) ? maze_state_up    : maze_state_stay;
    MazeState effective_maze_state_down  = ( down_unblocked_by_door || ! down_blocked_by_wall) ? maze_state_down  : maze_state_stay;

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
        switch(action) {
        case action_t::STAY:
            if(maze_state_to==effective_maze_state_stay ) prob += 1-epsilon;
            break;
        case action_t::LEFT:
            if(maze_state_to==effective_maze_state_left ) prob += 1-epsilon;
            break;
        case action_t::RIGHT:
            if(maze_state_to==effective_maze_state_right) prob += 1-epsilon;
            break;
        case action_t::UP:
            if(maze_state_to==effective_maze_state_up   ) prob += 1-epsilon;
            break;
        case action_t::DOWN:
            if(maze_state_to==effective_maze_state_down ) prob += 1-epsilon;
            break;
        default:
            DEBUG_OUT(0,"Error: unknown action");
            return 0;
        }

        // return if state transition impossible
        if(prob==0) {
            return 0;
        }
    }

    // calculate accumulated reward
    reward_t accumulated_reward = 0;
    for(auto r : rewards) {
        MazeState activate_state(r[ACTIVATION_STATE]);
        MazeState receive_state(r[RECEIVE_STATE]);
        idx_t delay = r[TIME_DELAY];
        REWARD_ACTIVATION_TYPE rat = (REWARD_ACTIVATION_TYPE)r[ACTIVATION_TYPE];

        // check if reward could be received
        bool receive_reward = false;
        if(maze_state_to==receive_state) {
            receive_reward = true;
        }

        // check if reward was activated (if it can be received or activation
        // type is with punishment for failure)
        if(receive_reward || rat==EACH_TIME_PUNISH_FAILURE || rat==ON_RELEASE_PUNISH_FAILURE) {
            idx_t steps_in_past = 1;
            bool reward_invalidated = false;
            for( const_instanceIt_t insIt = instance_from->const_it(); insIt!=INVALID && steps_in_past<=abs(delay); --insIt, ++steps_in_past) {

                // check if delay matches
                bool delay_matches = false;
                if(delay<0 || steps_in_past==delay) {
                // exact match for positve delay less-equal match for negative
                    delay_matches = true;
                }

                // check if agent was on activation state
                bool activation_state = false;
                if(MazeState(insIt->state)==activate_state) {
                    activation_state = true;
                }

                // handle different types of rewards
                switch(rat) {
                case EACH_TIME:
                    if(activation_state && receive_reward && delay_matches ) {
                        // receive reward whenever everything matches
                        accumulated_reward += r[REWARD_VALUE];
                    }
                    break;
                case ON_RELEASE:
                    if(activation_state && receive_reward && delay_matches && !reward_invalidated) {
                        // successfully receive reward
                        accumulated_reward += r[REWARD_VALUE];
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
                        } else {
                            accumulated_reward -= r[REWARD_VALUE];
                        }
                    }
                    break;
                case ON_RELEASE_PUNISH_FAILURE:
                    if(activation_state && delay_matches && !reward_invalidated) {
                        // receive reward when everything matches and reward was
                        // not invalidated, punish if not on receive-state
                        if(receive_reward) {
                            accumulated_reward += r[REWARD_VALUE];
                        } else {
                            accumulated_reward -= r[REWARD_VALUE];
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
    if(reward!=accumulated_reward) {
        return 0;
    }

    return prob;
}

void Maze::set_epsilon(const double& e) {
    epsilon = e;
}

void Maze::set_current_state(const state_t& state) {
    current_state = MazeState(state);
    for(idx_t k_idx=0; k_idx<(idx_t)Config::k; ++k_idx) {
        current_instance = current_instance->append_instance(action_t::STAY, current_state.state_idx(), reward_t(0));
    }
    DEBUG_OUT(1,"Set current state to (" << current_state.x() << "," << current_state.y() << ")");
}

std::string Maze::get_rewards() {
    std::stringstream ss;
    for(int r_idx=0; r_idx<(int)rewards.size(); ++r_idx) {
        ss << "Reward " << r_idx << std::endl;
        ss << "    ACTIVATION_STATE : " << (state_t)rewards[r_idx][ACTIVATION_STATE] << std::endl;
        ss << "    RECEIVE_STATE    : " << (state_t)rewards[r_idx][RECEIVE_STATE] << std::endl;
        ss << "    TIME_DELAY       : " << (int)rewards[r_idx][TIME_DELAY] << std::endl;
        ss << "    reward           : " << (reward_t)rewards[r_idx][REWARD_VALUE] << std::endl;
        ss << "    activation       : " << reward_activation_type_str((REWARD_ACTIVATION_TYPE)rewards[r_idx][ACTIVATION_TYPE]) << std::endl;
    }
    return ss.str();
}

std::string Maze::get_walls() {
    std::stringstream ss;
    for(int w_idx=0; w_idx<(int)walls.size(); ++w_idx) {
        ss << "Wall " << w_idx << ": (" << walls[w_idx][0] << "|" << walls[w_idx][1] << ")" << std::endl;
    }
    return ss.str();
}

void Maze::frame_maze() {

    QGraphicsScene * scene = view->scene();

    MazeState first_maze_state(stateIt_t::first());
    MazeState last_maze_state(stateIt_t::last());
    double border_x = first_maze_state.x()-state_size/2 - 0.1*state_size;
    double border_y = first_maze_state.y()-state_size/2 - 0.1*state_size;
    double border_width = last_maze_state.x()+state_size/2 - border_x + 0.1*state_size;
    double border_height = last_maze_state.y()+state_size/2 - border_y + 0.1*state_size;
    double reward_magnitude = util::max<double>(fabs(reward_t::max_reward), fabs(reward_t::min_reward));
    for(rewardIt_t rewIt=rewardIt_t::first(); rewIt!=INVALID; ++rewIt) {
        double intensity = ((reward_t)rewIt)/reward_magnitude;
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

void Maze::render_state(state_t s) {

    QGraphicsScene * scene = view->scene();

    QPen state_pen(QColor(0,0,0), 0.02, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    MazeState maze_state(s);
    QGraphicsRectItem * rect = scene->addRect( maze_state.x()-state_size/2,
                                               maze_state.y()-state_size/2,
                                               state_size,
                                               state_size,
                                               state_pen,
                                               QBrush(QColor(230,230,230))
        );
    state_rects.push_back(rect);
}

void Maze::render_wall(wall_t w) {

    QGraphicsScene * scene = view->scene();

    QPen wall_pen(QColor(50,50,50), 0.02, Qt::SolidLine, Qt::RoundCap);
    QBrush wall_brush(QColor(50,50,50));

    MazeState maze_state_1(w[0]);
    MazeState maze_state_2(w[1]);
    idx_t x_1 = maze_state_1.x();
    idx_t y_1 = maze_state_1.y();
    idx_t x_2 = maze_state_2.x();
    idx_t y_2 = maze_state_2.y();
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
        DEBUG_OUT(0,"Error: No wall possible between (" <<
                  x_1 << "," << y_1 << ") and (" <<
                  x_2 << "," << y_2 << ")" );
    }
}

void Maze::render_door(door_t dt) {

    QGraphicsScene * scene = view->scene();

    MazeState s1 = get<0>(dt);
    MazeState s2 = get<1>(dt);
    MazeState s3 = get<2>(dt);
    KEY_TYPE kt =  get<3>(dt);
    idx_t delay =  get<4>(dt);
    color_t c = get<5>(dt);

    QColor door_color(255*get<0>(c),255*get<1>(c),255*get<2>(c));
    QPen door_pen(door_color, 0.05, Qt::SolidLine, Qt::RoundCap);
    QBrush door_brush(door_color);

    // draw door
    idx_t x1 = s1.x();
    idx_t y1 = s1.y();
    idx_t x2 = s2.x();
    idx_t y2 = s2.y();
    if( abs(x1-x2)==1 && abs(y1-y2)==0 ) {
        idx_t x_min = min<idx_t>(x1,x2);
        idx_t y = y1;
        scene->addLine( x_min+0.5, y-0.3, x_min+0.7, y+0.3, door_pen );
    } else if( abs(x1-x2)==0 && abs(y1-y2)==1 ) {
        idx_t x = x1;
        idx_t y_min = min<idx_t>(y1,y2);
        scene->addLine( x+0.3, y_min+0.5, x-0.3, y_min+0.7, door_pen );
    } else {
        DEBUG_OUT(0,"Error: No door possible between (" <<
                  x1 << "," << y1 << ") and (" <<
                  x2 << "," << y2 << ")" );
    }

    // draw key
    idx_t x3 = s3.x();
    idx_t y3 = s3.y();
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
    QGraphicsTextItem * txt = scene->addText(
        QString("t=%1").arg(QString::number(delay)),
        QFont("",12)
        );
    QRectF box = txt->boundingRect();
    txt->setPos(
        text_x-text_scale*box.width()/2,
        text_y-text_scale*box.height()/2
        );
    txt->setScale(text_scale);
    txt->setDefaultTextColor(QColor(0,0,0));
}

void Maze::render_reward(maze_reward_t r) {

    QGraphicsScene * scene = view->scene();

    MazeState maze_state_1((int)r[ACTIVATION_STATE]);
    MazeState maze_state_2((int)r[RECEIVE_STATE]);
    double x_start   = maze_state_1.x();
    double y_start   = maze_state_1.y();
    double x_end     = maze_state_2.x();
    double y_end     = maze_state_2.y();
    double x_shift   = -(y_end-y_start)/5;
    double y_shift   = (x_end-x_start)/5;
    double x_control = (x_start+x_end)/2+x_shift;
    double y_control = (y_start+y_end)/2+y_shift;
    QColor color( (int)r[R], (int)r[G], (int)r[B]);
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
    switch((REWARD_ACTIVATION_TYPE)r[ACTIVATION_TYPE]) {
    case EACH_TIME:
        scene->addEllipse(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            no_brush
            );
        break;
    case EACH_TIME_PUNISH_FAILURE:
        scene->addEllipse(
            x_start-reward_start_size/2,
            y_start-reward_start_size/2,
            reward_start_size,
            reward_start_size,
            marker_pen,
            brush
            );
        break;
    case ON_RELEASE:
        scene->addRect(
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
            brush
            );
        break;
    default:
        DEBUG_DEAD_LINE;
    }


    // end
    double end_vector_x = x_end-x_control;
    double end_vector_y = y_end-y_control;
    double end_vector_norm = sqrt( pow(end_vector_x,2) + pow(end_vector_y,2) );
    end_vector_x /= end_vector_norm;
    end_vector_y /= end_vector_norm;
    double end_cross_vector_x = +end_vector_y;
    double end_cross_vector_y = -end_vector_x;
    QPainterPath end_path;
    end_path.moveTo(x_end, y_end);
    end_path.lineTo(
        x_end-reward_end_size*(end_vector_x+end_cross_vector_x*reward_end_ratio/2),
        y_end-reward_end_size*(end_vector_y+end_cross_vector_y*reward_end_ratio/2)
        );
    end_path.lineTo(
        x_end-reward_end_size*(end_vector_x-end_cross_vector_x*reward_end_ratio/2),
        y_end-reward_end_size*(end_vector_y-end_cross_vector_y*reward_end_ratio/2)
        );
    end_path.lineTo(x_end, y_end);
    scene->addPath(end_path,marker_pen,QBrush(color));

    // text
    double mid_point_x = x_start + (x_end - x_start)/2;
    double mid_point_y = y_start + (y_end - y_start)/2;
    idx_t time_delay = (idx_t)r[TIME_DELAY];
    reward_t reward = (reward_t)r[REWARD_VALUE];
    QGraphicsTextItem * txt = scene->addText(
        QString("t=%1,r=%2").arg(QString::number(time_delay)).arg(QString::number(reward)),
        QFont("",12)
        );
    QRectF box = txt->boundingRect();
    txt->setPos(
        x_control+(mid_point_x-x_control)*text_center-text_scale*box.width()/2,
        y_control+(mid_point_y-y_control)*text_center-text_scale*box.height()/2
        );
    txt->setScale(text_scale);
    txt->setDefaultTextColor(color);
}

void Maze::rescale_scene(QGraphicsView * view) {
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}

const char* Maze::reward_activation_type_str(REWARD_ACTIVATION_TYPE ra) {
    switch(ra) {
    case EACH_TIME:
        return "EACH_TIME";
    case ON_RELEASE:
        return "ON_RELEASE";
    case EACH_TIME_PUNISH_FAILURE:
        return "EACH_TIME_PUNISH_FAILURE";
    case ON_RELEASE_PUNISH_FAILURE:
        return "ON_RELEASE_PUNISH_FAILURE";
    default:
        DEBUG_DEAD_LINE;
        return "Error!";
    }
}
