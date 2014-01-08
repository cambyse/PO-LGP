#include "CheeseMaze.h"

#include "../util/QtUtil.h"
#include "../util/ColorOutput.h"

#define DEBUG_LEVEL 0
#include "../debug.h"

static const double state_size = 1;                  // Size of states for rendering.
static const double action_line_length_factor = 0.8; // How long the action line is relative to the state size.
static const double action_point_size_factor = 0.5;  // How large the action point is relative to the state size.

CheeseMaze::CheeseMaze():
    Environment(action_ptr_t(new CheeseMazeAction()),
                observation_ptr_t(new CheeseMazeObservation()),
                reward_ptr_t(new ListedReward({-1,-0.1,1},1))),
    mouse(nullptr),
    cheese(nullptr)
{}


CheeseMaze::~CheeseMaze() {
    delete mouse;
    delete cheese;
}

void CheeseMaze::render_initialize(QGraphicsView * v) {

    // intitialize view
    Visualizer::render_initialize(v);

    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();

    // Put frame around maze
    scene->addRect( -state_size/2 - 1,
                    -state_size/2 - 1,
                    5 + 2,
                    5 + 2,
                    QPen(QColor(0,0,0), 0.04, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin),
                    QBrush(QColor(255,255,255))
        );

    // Render state squares
    for(int state_idx=0; state_idx<=10; ++state_idx) {
        QPen state_pen(QColor(0,0,0), 0.07, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        scene->addRect( get_x_pos(state_idx)-state_size/2,
                        get_y_pos(state_idx)-state_size/2,
                        state_size,
                        state_size,
                        state_pen,
                        QBrush(QColor(255,255,255))
            );
    }

    // Render action line and circle
    {
        QPen action_line_pen(QColor(0,0,0,50),0.1,Qt::SolidLine,Qt::RoundCap);
        QPen action_point_pen(QColor(0,0,0),0.01,Qt::SolidLine,Qt::RoundCap);
        QBrush action_point_brush(QColor(0,0,0,30));
        action_line = scene->addLine(current_x_pos(),current_y_pos(),current_x_pos(),current_y_pos(),action_line_pen);
        double ap_size = state_size*action_point_size_factor;
        action_point = scene->addEllipse(current_x_pos()-ap_size/2,current_y_pos()-ap_size/2,ap_size,ap_size,action_point_pen,action_point_brush);
    }

    // render cheese
    {
        if(!cheese) {
            delete cheese;
        }
        cheese = new QGraphicsSvgItem("Images/cheese.svg");
        double scale = 0.3;
        cheese->setScale(scale);
        QSizeF s = cheese->boundingRect().size()*scale;
        cheese->setPos(get_x_pos(6)-s.width()/2, get_y_pos(6)-s.height()/2);
        scene->addItem(cheese);
    }

    // render mouse
    {
        if(!mouse) {
            delete mouse;
        }
        mouse = new QGraphicsSvgItem("Images/mouse.svg");
        double scale = 0.3;
        mouse->setScale(scale);
        QSizeF s = mouse->boundingRect().size()*scale;
        mouse->setPos(current_x_pos()-s.width()/2, current_y_pos()-s.height()/2);
        scene->addItem(mouse);
    }

    // render observations
    {
        double scale = 0.3;
        double x_pos = 2;
        double y_pos = 4;
        CheeseMazeObservation_N = new QGraphicsSvgItem("Images/CheeseMazeObservation_N.svg");
        QSizeF s = CheeseMazeObservation_N->boundingRect().size()*scale;
        CheeseMazeObservation_NS = new QGraphicsSvgItem("Images/CheeseMazeObservation_NS.svg");
        CheeseMazeObservation_NE = new QGraphicsSvgItem("Images/CheeseMazeObservation_NE.svg");
        CheeseMazeObservation_NW = new QGraphicsSvgItem("Images/CheeseMazeObservation_NW.svg");
        CheeseMazeObservation_EW = new QGraphicsSvgItem("Images/CheeseMazeObservation_EW.svg");
        CheeseMazeObservation_ESW = new QGraphicsSvgItem("Images/CheeseMazeObservation_ESW.svg");
        CheeseMazeObservation_N->setScale(scale);
        CheeseMazeObservation_NS->setScale(scale);
        CheeseMazeObservation_NE->setScale(scale);
        CheeseMazeObservation_NW->setScale(scale);
        CheeseMazeObservation_EW->setScale(scale);
        CheeseMazeObservation_ESW->setScale(scale);
        CheeseMazeObservation_N->setPos(x_pos-s.width()/2, y_pos-s.height()/2);
        CheeseMazeObservation_NS->setPos(x_pos-s.width()/2, y_pos-s.height()/2);
        CheeseMazeObservation_NE->setPos(x_pos-s.width()/2, y_pos-s.height()/2);
        CheeseMazeObservation_NW->setPos(x_pos-s.width()/2, y_pos-s.height()/2);
        CheeseMazeObservation_EW->setPos(x_pos-s.width()/2, y_pos-s.height()/2);
        CheeseMazeObservation_ESW->setPos(x_pos-s.width()/2, y_pos-s.height()/2);
        scene->addItem(CheeseMazeObservation_N);
        scene->addItem(CheeseMazeObservation_NS);
        scene->addItem(CheeseMazeObservation_NE);
        scene->addItem(CheeseMazeObservation_NW);
        scene->addItem(CheeseMazeObservation_EW);
        scene->addItem(CheeseMazeObservation_ESW);
    }

    rescale_scene(view);
}


void CheeseMaze::render_update() {
    // set mouse position and mirror to make 'stay' actions visible
    QSizeF s = mouse->boundingRect().size();
    mouse->setPos(current_x_pos()-mouse->scale()*s.width()/2, current_y_pos()-mouse->scale()*s.height()/2);

    // set action line and circle
    double al_length = state_size*action_line_length_factor;
    double ap_size = state_size*action_point_size_factor;
    action_point->setRect(last_x_pos()-ap_size/2,last_y_pos()-ap_size/2,ap_size,ap_size);
    if(last_action==action_t(action_t::ACTION::NORTH)) {
        action_line->setLine(last_x_pos(),last_y_pos(),last_x_pos(),last_y_pos()-al_length);
    } else if(last_action==action_t(action_t::ACTION::SOUTH)) {
        action_line->setLine(last_x_pos(),last_y_pos(),last_x_pos(),last_y_pos()+al_length);
    } else if(last_action==action_t(action_t::ACTION::WEST)) {
        action_line->setLine(last_x_pos(),last_y_pos(),last_x_pos()-al_length,last_y_pos());
    } else if(last_action==action_t(action_t::ACTION::EAST)) {
        action_line->setLine(last_x_pos(),last_y_pos(),last_x_pos()+al_length,last_y_pos());
    } else {
        DEBUG_ERROR("Invalid action (" << last_action << ")");
    }

    rescale_scene(view);
}

void CheeseMaze::render_tear_down() {
    view->scene()->clear();
    action_line = nullptr;
    action_point = nullptr;
    mouse = nullptr;
    cheese = nullptr;
    CheeseMazeObservation_N = nullptr;
    CheeseMazeObservation_NE = nullptr;
    CheeseMazeObservation_NS = nullptr;
    CheeseMazeObservation_NW = nullptr;
    CheeseMazeObservation_EW = nullptr;
    CheeseMazeObservation_ESW = nullptr;
}

void CheeseMaze::perform_transition(const action_ptr_t& action) {
    observation_ptr_t o;
    reward_ptr_t r;
    perform_transition(action, o, r);
}

void CheeseMaze::perform_transition(const action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r ) {
    // remember old state
    last_state_idx = current_state_idx;
    // cast given action
    auto action = a.get_derived<const CheeseMazeAction>();
    last_action = *action;
    // determine new state
    switch(current_state_idx) {
    case 0:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::EAST:
            current_state_idx = 3;
            break;
        case CheeseMazeAction::ACTION::SOUTH:
            current_state_idx = 1;
            break;
        default:
            break;
        }
        break;
    case 1:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::NORTH:
            current_state_idx = 0;
            break;
        case CheeseMazeAction::ACTION::SOUTH:
            current_state_idx = 2;
            break;
        default:
            break;
        }
        break;
    case 2:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::NORTH:
            current_state_idx = 1;
            break;
        default:
            break;
        }
        break;
    case 3:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::EAST:
            current_state_idx = 4;
            break;
        case CheeseMazeAction::ACTION::WEST:
            current_state_idx = 0;
            break;
        default:
            break;
        }
        break;
    case 4:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::EAST:
            current_state_idx = 7;
            break;
        case CheeseMazeAction::ACTION::SOUTH:
            current_state_idx = 5;
            break;
        case CheeseMazeAction::ACTION::WEST:
            current_state_idx = 3;
            break;
        default:
            break;
        }
        break;
    case 5:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::NORTH:
            current_state_idx = 4;
            break;
        case CheeseMazeAction::ACTION::SOUTH:
            current_state_idx = 6;
            break;
        default:
            break;
        }
        break;
    case 6:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::NORTH:
            current_state_idx = 5;
            break;
        default:
            break;
        }
        break;
    case 7:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::EAST:
            current_state_idx = 8;
            break;
        case CheeseMazeAction::ACTION::WEST:
            current_state_idx = 4;
            break;
        default:
            break;
        }
        break;
    case 8:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::SOUTH:
            current_state_idx = 9;
            break;
        case CheeseMazeAction::ACTION::WEST:
            current_state_idx = 7;
            break;
        default:
            break;
        }
        break;
    case 9:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::NORTH:
            current_state_idx = 8;
            break;
        case CheeseMazeAction::ACTION::SOUTH:
            current_state_idx = 10;
            break;
        default:
            break;
        }
        break;
    case 10:
        switch(action->get_action()) {
        case CheeseMazeAction::ACTION::NORTH:
            current_state_idx = 9;
            break;
        default:
            break;
        }
        break;
    default:
        DEBUG_DEAD_LINE;
        DEBUG_OUT(0,"current state index: " << current_state_idx);
    }
    // determine reward
    if(current_state_idx==6) {
        r = reward_space.get_derived<const ListedReward>()->new_reward(1.0);
        current_state_idx = rand()%11;
    } else if(current_state_idx==last_state_idx) {
        r = reward_space.get_derived<const ListedReward>()->new_reward(-1.0);
    } else {
        r = reward_space.get_derived<const ListedReward>()->new_reward(-0.1);
    }
    // determine observation
    o = get_observation(current_state_idx);
}

void CheeseMaze::get_features(std::vector<f_ptr_t> & basis_features, FeatureLearner::LEARNER_TYPE type) const {

    // clear first
    basis_features.clear();

#warning hack: k is not fixed!
    int k = 5;

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

int CheeseMaze::get_x_pos(int state_idx) {
    switch(state_idx) {
    case 0:
        return 0;
    case 1:
        return 0;
    case 2:
        return 0;
    case 3:
        return 1;
    case 4:
        return 2;
    case 5:
        return 2;
    case 6:
        return 2;
    case 7:
        return 3;
    case 8:
        return 4;
    case 9:
        return 4;
    case 10:
        return 4;
    default:
        DEBUG_DEAD_LINE;
        return 0;
    }
}

int CheeseMaze::get_y_pos(int state_idx) {
    switch(state_idx) {
    case 0:
        return 0;
    case 1:
        return 1;
    case 2:
        return 2;
    case 3:
        return 0;
    case 4:
        return 0;
    case 5:
        return 1;
    case 6:
        return 2;
    case 7:
        return 0;
    case 8:
        return 0;
    case 9:
        return 1;
    case 10:
        return 2;
    default:
        DEBUG_DEAD_LINE;
        return 0;
    }
}

int CheeseMaze::current_x_pos() const {
    return get_x_pos(current_state_idx);
}

int CheeseMaze::current_y_pos() const {
    return get_y_pos(current_state_idx);
}

int CheeseMaze::last_x_pos() const {
    return get_x_pos(last_state_idx);
}

int CheeseMaze::last_y_pos() const {
    return get_y_pos(last_state_idx);
}

CheeseMaze::observation_ptr_t CheeseMaze::get_observation(int state_idx) const {
    switch(state_idx) {
    case 0:
        return observation_ptr_t(new CheeseMazeObservation("NW"));
    case 1:
        return observation_ptr_t(new CheeseMazeObservation("EW"));
    case 2:
        return observation_ptr_t(new CheeseMazeObservation("ESW"));
    case 3:
        return observation_ptr_t(new CheeseMazeObservation("NS"));
    case 4:
        return observation_ptr_t(new CheeseMazeObservation("N"));
    case 5:
        return observation_ptr_t(new CheeseMazeObservation("EW"));
    case 6:
        return observation_ptr_t(new CheeseMazeObservation("ESW"));
    case 7:
        return observation_ptr_t(new CheeseMazeObservation("NS"));
    case 8:
        return observation_ptr_t(new CheeseMazeObservation("NE"));
    case 9:
        return observation_ptr_t(new CheeseMazeObservation("EW"));
    case 10:
        return observation_ptr_t(new CheeseMazeObservation("ESW"));
    default:
        DEBUG_DEAD_LINE;
        return observation_ptr_t(new CheeseMazeObservation());
    }
}
