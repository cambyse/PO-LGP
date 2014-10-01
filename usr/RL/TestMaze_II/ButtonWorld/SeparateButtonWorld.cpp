#include "SeparateButtonWorld.h"

#include "../Representation/DoublyLinkedInstance.h"

#define DEBUG_LEVEL 1
#include "../util/debug.h"

using std::vector;

static const double button_size = 0.9;
static const QPen button_pen_OFF(QColor(0,0,0),0.03,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
static const QPen button_pen_ON(QColor(0,0,0),0.1,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
static const QBrush reward_brush_NONE(QColor(200, 200, 200));
static const QBrush reward_brush_PLUS(QColor(0, 200, 0));
static const QBrush reward_brush_MINUS(QColor(200, 0, 0));
static const QPen reward_pen(QColor(0,0,0),0.03,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin);
static const double text_scale = 0.008;
static const QFont  text_font = QFont("Helvetica [Cronyx]", 12);

SeparateButtonWorld::SeparateButtonWorld(int s, std::vector<probability_t> p):
    ButtonWorld(s,p),
    last_action(size),
    last_reward(size, 0)
{

    // set rewards
    vector<double> reward_list;
    for(int rew : util::Range(-size, size)) {
        reward_list.push_back(rew);
    }
    set_spaces(action_ptr_t(new action_t(s)),
               observation_ptr_t(new observation_t),
               reward_ptr_t(new reward_t(reward_list, 0.)));
    current_instance = DoublyLinkedInstance::create(action_space, observation_space, reward_space);
}

SeparateButtonWorld::SeparateButtonWorld(int s, double alpha):
    SeparateButtonWorld(s, SeparateButtonWorld::probs_from_beta(s, alpha)) {}

void SeparateButtonWorld::render_initialize(QGraphicsView * v) {
    // intitialize view
    Visualizer::render_initialize(v);

    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();

    // Render buttons/rewards/text
    button_array.clear();
    reward_array.clear();
    for(auto idx_prob : util::enumerate(prob_array)) {
        // buttons
        QBrush button_brush;
        if(idx_prob.second>=0.5) {
            double scale = 1-2*(idx_prob.second-0.5);
            button_brush = QBrush(QColor(scale*200, 200, scale*200));
        } else {
            double scale = 2*(idx_prob.second);
            button_brush = QBrush(QColor(200, scale*200, scale*200));
        }
        button_array.push_back(scene->addRect(idx_prob.first,
                                              0,
                                              button_size,
                                              button_size,
                                              button_pen_OFF,
                                              button_brush));
        // rewards
        reward_array.push_back(scene->addEllipse(idx_prob.first,
                                                 1,
                                                 button_size,
                                                 button_size,
                                                 reward_pen,
                                                 reward_brush_NONE));
        // texts
        QGraphicsTextItem * txt = scene->addText(
            QString("%1").arg(idx_prob.second, 5, 'f', 3),
            text_font
            );
        QRectF box = txt->boundingRect();
        txt->setPos(
            idx_prob.first+button_size/2-text_scale*box.width()/2,
            -1+button_size/2-text_scale*box.height()/2
            );
        txt->setScale(text_scale);
    }

    // put frame around everything
    scene->addRect(-0.5, -1.5, size+1, 4, button_pen_OFF );

    rescale_scene();
}

void SeparateButtonWorld::render_update() {
    for(int idx = 0; idx < (int)last_action.get_array().size(); ++idx) {
        if(last_action.get_array()[idx]) {
            button_array[idx]->setPen(button_pen_ON);
        } else {
            button_array[idx]->setPen(button_pen_OFF);
        }
        switch(last_reward[idx]) {
        case 1:
            reward_array[idx]->setBrush(reward_brush_PLUS);
            break;
        case -1:
            reward_array[idx]->setBrush(reward_brush_MINUS);
            break;
        case 0:
            reward_array[idx]->setBrush(reward_brush_NONE);
            break;
        default:
            DEBUG_DEAD_LINE;
        }
    }
    rescale_scene();
}

void SeparateButtonWorld::render_tear_down() {
    Visualizer::render_tear_down();
    button_array.assign(0, nullptr);
    reward_array.assign(0, nullptr);
}

void SeparateButtonWorld::perform_transition(const action_ptr_t& action) {
    action_t this_button_action = *(action.get_derived<action_t>());
    for(auto idx_rew : util::enumerate(last_reward)) {
        if(last_action.get_array()[idx_rew.first] && this_button_action.get_array()[idx_rew.first]) {
            if(drand48() < prob_array[idx_rew.first]) {
                idx_rew.second = 1;
            } else {
                idx_rew.second = -1;
            }
        } else {
            idx_rew.second = 0;
        }
    }
    last_action = this_button_action;

    reward_t::value_t sum = 0;
    for(auto rew : last_reward) {
        sum += rew;
    }

    current_instance = current_instance->append(action, observation_space, reward_space.get_derived<reward_t>()->new_reward(sum));
}

SeparateButtonWorld::probability_t SeparateButtonWorld::get_prediction(const_instance_ptr_t ins,
                                                       const action_ptr_t& action,
                                                       const observation_ptr_t&,
                                                       const reward_ptr_t& reward) const {
    // convert to button actions
    action_t last_button_action(size);
    if(ins!=util::INVALID) {
        last_button_action = *(ins->action.get_derived<action_t>());
    }
    action_t next_button_action = *(action.get_derived<action_t>());
    // get array of probs for buttons pushed twice in a row
    vector<probability_t> active_probs;
    for(auto idx_prob : util::enumerate(prob_array)) {
        if(last_button_action.get_array()[idx_prob.first] &&
           last_button_action.get_array()[idx_prob.first]) {
            active_probs.push_back(idx_prob.second);
        }
    }
    // get reward
    int rew = reward->get_value();
    if(abs(rew)>(int)active_probs.size() || abs(rew)%2 != (int)active_probs.size()%2) {
        // return if impossible to achieve
        return 0;
    }
    // get number of PLUS/MINUS rewards
    int n_plus = 0, n_minus = 0;
    if(rew>0) {
        n_plus += rew;
        n_plus += (active_probs.size()-rew)/2;
        n_minus = active_probs.size() - n_plus;
    } else {
        n_minus += abs(rew);
        n_minus += (active_probs.size()-abs(rew))/2;
        n_plus = active_probs.size() - n_minus;
    }
    // construct vector of bool indicating which buttons return PLUS and which MINUS
    vector<bool> plus_minus(n_plus, true);
    plus_minus.resize(n_plus+n_minus, false);
    // go through all possible orderings
    std::sort(plus_minus.begin(), plus_minus.end());
    probability_t prob = 0;
    do {
        probability_t this_prob = 1;
        for(auto idx_prob : util::enumerate(active_probs)) {
            if(plus_minus[idx_prob.first]) {
                this_prob *= idx_prob.second;
            } else {
                this_prob *= 1 - idx_prob.second;
            }
        }
        prob += this_prob;
    } while(std::next_permutation(plus_minus.begin(), plus_minus.end()));
    return prob;
}
