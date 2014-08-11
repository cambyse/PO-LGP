#include "ButtonWorld.h"

#include "../util/debug.h"

using std::vector;

ButtonWorld::ButtonWorld(int s, std::vector<probability_t> p):
    PredictiveEnvironment(action_ptr_t(), observation_ptr_t(), reward_ptr_t()),
    size(s),
    reward_probs(p),
    last_action(size)
{
    if(reward_probs.size() == 0) {
        DEBUG_WARNING("Empty probabilities, initializing to 0.5");
        reward_probs.assign(size, 0.5);
    }
    vector<double> reward_list;
    for(int rew : util::Range(-size, size)) {
        reward_list.push_back(rew);
    }
    set_spaces(action_ptr_t(new ButtonAction(s)),
               observation_ptr_t(),
               reward_ptr_t(new ListedReward(reward_list, 0.)));
}

void ButtonWorld::render_initialize(QGraphicsView * v) {
    // intitialize view
    Visualizer::render_initialize(v);

    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();

    // Render buttons
    QPen action_line_pen(QColor(0,0,0,50),0.1,Qt::SolidLine,Qt::RoundCap);
    QPen action_point_pen(QColor(0,0,0),0.01,Qt::SolidLine,Qt::RoundCap);
    QBrush action_point_brush(QColor(0,0,0,30));
    for(probability_t prob : reward_probs) {
        scene->addEllipse(-1, -1, 2, 2, action_point_pen, action_point_brush);
    }

    rescale_scene(view);
}

void ButtonWorld::render_update() {
    DEBUG_WARNING("RENDER UPDATE");
}

void ButtonWorld::render_tear_down() {
    Visualizer::render_tear_down();
}

void ButtonWorld::perform_transition(const action_ptr_t& action) {
    last_action = *(action.get_derived<action_t>());
}

ButtonWorld::probability_t ButtonWorld::get_prediction(const_instance_ptr_t,
                                                       const action_ptr_t&,
                                                       const observation_ptr_t&,
                                                       const reward_ptr_t&) const {
    probability_t prob = 1;
    DEBUG_WARNING("Not really implemented");
    return prob;
}

void ButtonWorld::get_features(std::vector<f_ptr_t> & basis_features,
                               FeatureLearner::LEARNER_TYPE type) const {

    // clear first
    basis_features.clear();

    #warning To be implemented...

}
