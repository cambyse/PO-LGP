#include "CheeseMaze.h"

#include "../util/QtUtil.h"
#include "../util/ColorOutput.h"

#define DEBUG_LEVEL 0
#include "../debug.h"

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


CheeseMaze::CheeseMaze():
    Environment(action_ptr_t(new CheeseMazeAction()),
                observation_ptr_t(new CheeseMazeObservation()),
                reward_ptr_t(new ListedReward({-1,0,1},1))),
    mouse(nullptr)
{}


CheeseMaze::~CheeseMaze() {
    delete mouse;
}

void CheeseMaze::render_initialize(QGraphicsView * v) {

    // intitialize view
    Visualizer::render_initialize(v);

    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();

    // Render action line and circle
    QPen action_line_pen(QColor(0,0,0,50),0.1,Qt::SolidLine,Qt::RoundCap);
    QPen action_point_pen(QColor(0,0,0),0.01,Qt::SolidLine,Qt::RoundCap);
    QBrush action_point_brush(QColor(0,0,0,30));
    action_line = scene->addLine(current_x_pos,current_y_pos,current_x_pos,current_y_pos,action_line_pen);
    double ap_size = state_size*action_point_size_factor;
    action_point = scene->addEllipse(current_x_pos-ap_size/2,current_y_pos-ap_size/2,ap_size,ap_size,action_point_pen,action_point_brush);

    // render mouse
    if(!mouse) {
        delete mouse;
    }
    mouse = new QGraphicsSvgItem("mouse.svg");
    mouse->setScale(0.2);
    QSizeF s = mouse->boundingRect().size();
    mouse->setPos(current_x_pos-s.width()/2, current_y_pos-s.height()/2);

    scene->addItem(mouse);

    rescale_scene(view);
}


void CheeseMaze::render_update() {
    // set mouse position and mirror to make 'stay' actions visible
    QSizeF s = mouse->boundingRect().size();
    mouse->setPos(current_x_pos-mouse->scale()*s.width()/2, current_y_pos-mouse->scale()*s.height()/2);

    // set action line and circle
    double al_length = state_size*action_line_length_factor;
    double ap_size = state_size*action_point_size_factor;
    action_point->setRect(last_x_pos-ap_size/2,last_y_pos-ap_size/2,ap_size,ap_size);
    if(last_action==action_t(action_t::ACTION::NORTH)) {
        action_line->setLine(last_x_pos,last_y_pos,last_x_pos,last_y_pos-al_length);
    } else if(last_action==action_t(action_t::ACTION::SOUTH)) {
        action_line->setLine(last_x_pos,last_y_pos,last_x_pos,last_y_pos+al_length);
    } else if(last_action==action_t(action_t::ACTION::WEST)) {
        action_line->setLine(last_x_pos,last_y_pos,last_x_pos-al_length,last_y_pos);
    } else if(last_action==action_t(action_t::ACTION::EAST)) {
        action_line->setLine(last_x_pos,last_y_pos,last_x_pos+al_length,last_y_pos);
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
}

void CheeseMaze::perform_transition(const action_ptr_t& action) {
#warning todo
}

void CheeseMaze::perform_transition(const action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r ) {
#warning todo
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
