#include "JointButtonWorld.h"

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

JointButtonWorld::JointButtonWorld(int s, std::vector<probability_t> p):
    ButtonWorld(s,p),
    last_action(new action_t(size)),
    last_reward(new reward_t({-1,1}, 0))
{
    // set action, observation, reward space
    set_spaces(last_action,
               observation_ptr_t(new observation_t),
               last_reward);
    // init current instance
    current_instance = DoublyLinkedInstance::create(action_space, observation_space, reward_space);
}

JointButtonWorld::JointButtonWorld(int s, double alpha):
    JointButtonWorld(s, JointButtonWorld::probs_from_beta(s, alpha)) {}

void JointButtonWorld::render_initialize(QGraphicsView * v) {
    // intitialize view
    Visualizer::render_initialize(v);

    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();

    // Render buttons/text
    button_array.clear();
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
    // reward
    reward_item = scene->addEllipse((double)(button_array.size()-1)/2,
                                    1,
                                    button_size,
                                    button_size,
                                    reward_pen,
                                    reward_brush_NONE);

    // put frame around everything
    scene->addRect(-0.5, -1.5, size+1, 4, button_pen_OFF );

    rescale_scene();
}

void JointButtonWorld::render_update() {
    auto action_array = last_action.get_derived<action_t>()->get_array();
    for(int idx = 0; idx < (int)action_array.size(); ++idx) {
        if(action_array[idx]) {
            button_array[idx]->setPen(button_pen_ON);
        } else {
            button_array[idx]->setPen(button_pen_OFF);
        }
    }
    switch((int)last_reward->get_value()) {
    case 1:
        reward_item->setBrush(reward_brush_PLUS);
        break;
    case -1:
        reward_item->setBrush(reward_brush_MINUS);
        break;
    case 0:
        reward_item->setBrush(reward_brush_NONE);
        break;
    default:
        DEBUG_DEAD_LINE;
    }
    rescale_scene();
}

void JointButtonWorld::render_tear_down() {
    Visualizer::render_tear_down();
    button_array.assign(0, nullptr);
    reward_item = nullptr;
}

void JointButtonWorld::perform_transition(const action_ptr_t& this_action) {
    // get button actions
    action_t this_button_action = *(this_action.get_derived<action_t>());
    action_t last_button_action = *(last_action.get_derived<action_t>());
    auto this_pushed = this_button_action.get_array();
    auto last_pushed = last_button_action.get_array();
    // get prob and select reward
    probability_t prob = prob_from_arrays(last_pushed, this_pushed);
    if(drand48() < prob) {
        last_reward = *++reward_space.begin(); // second elem from reward space
    } else {
        last_reward = *reward_space.begin(); // first elem from reward space
    }
    // update action
    last_action = this_action;
    // update current instance
    current_instance = current_instance->append(last_action, observation_space, last_reward);
}

JointButtonWorld::probability_t JointButtonWorld::get_prediction(const_instance_ptr_t ins,
                                                       const action_ptr_t& action,
                                                       const observation_ptr_t&,
                                                       const reward_ptr_t& reward) const {
    // convert to button actions
    action_t last_button_action(size);
    if(ins!=util::INVALID) {
        last_button_action = *(ins->action.get_derived<action_t>());
    }
    action_t next_button_action = *(action.get_derived<action_t>());

    // get arrays of pushed buttons
    auto last_pushed = last_button_action.get_array();
    auto next_pushed = next_button_action.get_array();

    // get prob from arrays of pushed buttons
    probability_t prob = prob_from_arrays(last_pushed, next_pushed);

    // return
    if(reward==*++reward_space.begin()) { // second elem from reward space
        return prob;
    } else if(reward==*reward_space.begin()) { // first elem from reward space
        return 1-prob;
    } else {
        DEBUG_DEAD_LINE;
        return 0;
    }
}

JointButtonWorld::probability_t JointButtonWorld::prob_from_arrays(const vector<bool> & last_pushed,
                                                         const vector<bool> & this_pushed) const {
    probability_t prob = 1;
    for(int idx : util::Range(size)) {
        if(last_pushed[idx]==this_pushed[idx]) {
            prob *= prob_array[idx];
        } else {
            prob *= 1 - prob_array[idx];
        }
    }
    return prob;
}
