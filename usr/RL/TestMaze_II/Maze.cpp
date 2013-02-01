
#include "Maze.h"

#define DEBUG_LEVEL 1
#include "debug.h"

const double Maze::state_size = 0.9;

Maze::Maze(const double& eps):
        time_delay(Data::k),
        reward_active(false),
        epsilon(eps),
        agent(NULL), button(NULL), smiley(NULL) {

    if(time_delay<=0) {
        DEBUG_OUT(0,"Error: Time delay must be larger than zero --> setting to one");
        time_delay = 1;
    }

    // setting button and smiley state
    if(Data::maze_x_dim>0 || Data::maze_y_dim>0) {
        button_state = MazeState(Data::maze_x_dim-1,Data::maze_y_dim-1);
    } else {
        button_state = MazeState(0,0);
    }
    smiley_state = MazeState(0,0);

    // setting current state
    current_state = MazeState(Data::maze_x_dim/2, Data::maze_y_dim/2);
    for(idx_t k_idx=0; k_idx<(idx_t)Data::k; ++k_idx) current_k_mdp_state.new_state(Data::STAY, current_state.state_idx(), Data::min_reward);
}


Maze::~Maze() {
    delete agent;
    delete button;
    delete smiley;
}


void Maze::render_initialize(QGraphicsView * view) {
    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();
    if(scene==NULL) {
        scene = new QGraphicsScene();
        view->setScene(scene);
    }

    for(state_t state=0; state<(idx_t)Data::state_n; ++state) {
        MazeState maze_state(state);
        scene->addRect( maze_state.x()-state_size/2, maze_state.y()-state_size/2, state_size, state_size, QPen(QColor(0,0,0)), QBrush(QColor(230,230,230)) );
    }

    // initialize and render button state
    if(!button) {
        button = new QGraphicsSvgItem("./button.svg");
        QSizeF s = button->boundingRect().size();
        button->setTransformOriginPoint(s.width()/2,s.height()/2);
        button->setScale(0.2);
        button->setPos(button_state.x()-s.width()/2,button_state.y()-s.height()/2);
    }
    button->setElementId(current_state==button_state ? "active" : "passive");
    scene->addItem(button);

    // initialize and render smiley state
    if(!smiley) {
        smiley = new QGraphicsSvgItem("./smiley.svg");
        QSizeF s = smiley->boundingRect().size();
        smiley->setTransformOriginPoint(s.width()/2,s.height()/2);
        smiley->setScale(0.2);
        smiley->setPos(smiley_state.x()-s.width()/2, smiley_state.y()-s.height()/2);
    }
    smiley->setElementId( reward_active ? "active" : "passive");
    scene->addItem(smiley);



    // walls
    DEBUG_OUT(1,"Initializing " << walls_n << " walls" );
    for(idx_t idx=0; idx<(idx_t)walls_n; ++idx) {
        MazeState maze_state_1(Data::state_from_idx(walls[idx][0]));
        MazeState maze_state_2(Data::state_from_idx(walls[idx][1]));
        if(
                abs(maze_state_1.x()-maze_state_2.x())>1 ||
                abs(maze_state_1.y()-maze_state_2.y())>1 ||
                abs(maze_state_1.x()-maze_state_2.x())+abs(maze_state_1.y()-maze_state_2.y())>1
        ) {
            DEBUG_OUT(0,"Error: No wall possible between (" <<
                    maze_state_1.x() << "," << maze_state_1.y() << ") and (" <<
                    maze_state_2.y() << "," << maze_state_2.y() << ")" );

        }
    }






    // render agent
    if(!agent) {
        agent = new QGraphicsSvgItem("./agent.svg");
        QSizeF s = agent->boundingRect().size();
        agent->setTransformOriginPoint(s.width()/2,s.height()/2);
        agent->setScale(0.2);
        agent->setPos(current_state.x()-s.width()/2, current_state.y()-s.height()/2);
    }

    scene->addItem(agent);

    rescale_scene(view);
}


void Maze::render_update(QGraphicsView * view) {
    button->setElementId(current_state==button_state ? "active" : "passive");
    smiley->setElementId( reward_active ? "active" : "passive");
    QSizeF s = agent->boundingRect().size();
    agent->setPos(current_state.x()-s.width()/2, current_state.y()-s.height()/2);
    rescale_scene(view);
}


void Maze::perform_transition(const action_t& action) {
    MazeState old_state = current_state; // remember current (old) state

    // perform transition
    probability_t prob_threshold = drand48();
    DEBUG_OUT(2,"Prob threshold = " << prob_threshold);
    probability_t prob_accum = 0;
    bool was_set = false;
    for(Data::state_idx_t state_idx_to=0; state_idx_to<(idx_t)Data::state_n && !was_set; ++state_idx_to) {
        state_t state_to = Data::state_from_idx(state_idx_to);
        for(Data::reward_idx_t reward_idx=0; reward_idx<(idx_t)Data::reward_n && !was_set; ++reward_idx) {
            reward_t reward = Data::reward_from_idx(reward_idx);
            probability_t prob = get_prediction(current_k_mdp_state.get_k_mdp_state(), action, state_to, reward);
            DEBUG_OUT(2,"state(" << state_to << "), reward(" << reward << ") --> prob=" << prob);
            prob_accum += prob;
            if(prob_accum>prob_threshold) {
                reward_active = current_k_mdp_state.get_k_mdp_state()[time_delay-1].state==button_state.state_idx();
                current_state = state_to;
                current_k_mdp_state.new_state(action, state_to, reward);
                was_set = true;
                DEBUG_OUT(2,"CHOOSE");
            }
        }
    }
    if(!was_set) {
        DEBUG_OUT(0, "Error: Unnormalized probabilities [sum(p)=" << prob_accum << "]--> no transition performed." );
    }

    DEBUG_OUT(1, "(" << old_state.x() << "," << old_state.y() << ") + " << Data::action_strings[action] << " ==> (" << current_state.x() << "," << current_state.y() << ")");
}


void Maze::perform_transition(const action_t& a, Data::state_t& final_state, reward_t& r) {
    perform_transition(a);
    final_state = current_state.state_idx();
    r = current_k_mdp_state.get_k_mdp_state()[0].reward;
}

void Maze::initialize_predictions(QIteration& predictions) {
    unsigned long counter = 0;
    for(Data::k_mdp_state_idx_t k_mdp_state_idx_from=0; k_mdp_state_idx_from<(idx_t)Data::k_mdp_state_n; ++k_mdp_state_idx_from) {
        Data::k_mdp_state_t k_mdp_state_from = Data::k_mdp_state_from_idx(k_mdp_state_idx_from);
        for(Data::action_idx_t action_idx = 0; action_idx<(idx_t)Data::action_n; ++action_idx) {
            action_t action = Data::action_from_idx(action_idx);
            for(Data::state_idx_t state_to_idx=0; state_to_idx<(idx_t)Data::state_n; ++state_to_idx) {
                state_t state_to = Data::state_from_idx(state_to_idx);
                for(Data::reward_idx_t reward_idx=0; reward_idx<(idx_t)Data::reward_n; ++reward_idx) {
                    reward_t reward = Data::reward_from_idx(reward_idx);
                    predictions.set_prediction(
                            k_mdp_state_from,
                            action,
                            state_to,
                            reward,
                            get_prediction(k_mdp_state_from, action, state_to, reward)
                    );
                    ++counter;
                }
            }
        }
    }
    DEBUG_OUT(1,"Initialized " << counter << " predictions");
}

Maze::probability_t Maze::get_prediction(const k_mdp_state_t& k_mdp_state_from, const action_t& action, const state_t& state_to, const reward_t& reward) const {

    // check for matching reward
    if(k_mdp_state_from[time_delay-1].state==button_state.state_idx()
            && state_to==smiley_state.state_idx()
            && reward!=Data::max_reward ) {
        return 0;
    } else if( ( k_mdp_state_from[time_delay-1].state!=button_state.state_idx() || state_to!=smiley_state.state_idx() )
            && reward==Data::max_reward ) {
        return 0;
    }

    // check for matching state
    MazeState maze_state_to( state_to );
    MazeState state_from( k_mdp_state_from[0].state);
    MazeState state_left( clamp(0,Data::maze_x_dim-1,state_from.x()-1),clamp(0,Data::maze_y_dim-1,state_from.y()  ));
    MazeState state_right(clamp(0,Data::maze_x_dim-1,state_from.x()+1),clamp(0,Data::maze_y_dim-1,state_from.y()  ));
    MazeState state_up(   clamp(0,Data::maze_x_dim-1,state_from.x()  ),clamp(0,Data::maze_y_dim-1,state_from.y()-1));
    MazeState state_down( clamp(0,Data::maze_x_dim-1,state_from.x()  ),clamp(0,Data::maze_y_dim-1,state_from.y()+1));

    probability_t prob = 0;

    if(maze_state_to==state_from ) prob += epsilon/5;
    if(maze_state_to==state_left ) prob += epsilon/5;
    if(maze_state_to==state_right) prob += epsilon/5;
    if(maze_state_to==state_up   ) prob += epsilon/5;
    if(maze_state_to==state_down ) prob += epsilon/5;

    switch(action) {
    case Data::STAY:
        if(maze_state_to==state_from ) prob += 1-epsilon;
        break;
    case Data::LEFT:
        if(maze_state_to==state_left ) prob += 1-epsilon;
        break;
    case Data::RIGHT:
        if(maze_state_to==state_right) prob += 1-epsilon;
        break;
    case Data::UP:
        if(maze_state_to==state_up   ) prob += 1-epsilon;
        break;
    case Data::DOWN:
        if(maze_state_to==state_down ) prob += 1-epsilon;
        break;
    default:
        DEBUG_OUT(0,"Error: unknown action");
        return 0;
    }

    return prob;
}

void Maze::set_time_delay(const int& new_time_delay) {
    if(new_time_delay>(idx_t)Data::k) {
        DEBUG_OUT(0,"Error: Reward time delays larger than history length 'k' not allowed");
    } else if(new_time_delay<0){
        DEBUG_OUT(0,"Error: Time delay must be larger than zero --> keeping old value");
    } else {
        time_delay = new_time_delay;
    }
}

void Maze::set_epsilon(const double& e) {
    epsilon = e;
}

void Maze::set_current_state(const state_t& state) {
    current_state = MazeState(state);
    for(idx_t k_idx=0; k_idx<(idx_t)Data::k; ++k_idx) current_k_mdp_state.new_state(Data::STAY, current_state.state_idx(), Data::min_reward);
    DEBUG_OUT(1,"Set current state to (" << current_state.x() << "," << current_state.y() << ")");
}

void Maze::rescale_scene(QGraphicsView * view) {
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}
