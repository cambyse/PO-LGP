#ifndef BUTTONWORLD_H_
#define BUTTONWORLD_H_

#include "../PredictiveEnvironment.h"
#include "../Visualizer.h"

#include <QGraphicsSvgItem>

#include "../Config.h"
#include "../util/util.h"
#include "../Representation/Feature.h"
#include "../Representation/ListedReward.h"
#include "../Representation/AbstractObservation.h"
#include "ButtonAction.h"
#include "ButtonObservation.h"

class ButtonWorld: public PredictiveEnvironment, public Visualizer  {

    //----typedefs/classes----//
public:
    USE_CONFIG_TYPEDEFS;
    typedef ButtonAction action_t;
    typedef ButtonObservation observation_t;
    typedef ListedReward reward_t;

    //----members----//
public:
    const int size;
    static const bool use_factored_action_features = true;
protected:
    std::vector<probability_t> button_probs;
    action_ptr_t last_action;
    std::vector<QGraphicsRectItem*> button_array;
    QGraphicsEllipseItem* reward_item;
    reward_ptr_t last_reward;

    //----methods----//
public:
    ButtonWorld(int s = 1, std::vector<probability_t> p = std::vector<probability_t>());
    ButtonWorld(int s, double alpha);
    virtual ~ButtonWorld() override = default;
    virtual void render_initialize(QGraphicsView * v) override;
    virtual void render_update() override;
    virtual void render_tear_down() override;
    virtual void perform_transition(const action_ptr_t& action) override;
    probability_t get_prediction(const_instance_ptr_t,
                                 const action_ptr_t&,
                                 const observation_ptr_t&,
                                 const reward_ptr_t&) const override;
    virtual void get_features(f_set_t & basis_features,
                              FeatureLearner::LEARNER_TYPE type) const override;
    static void construct_factored_action_features(f_set_t & basis_features,
                                                   action_ptr_t action,
                                                   int delay);
private:
    virtual probability_t prob_from_arrays(const std::vector<bool> & last_pushed,
                                           const std::vector<bool> & this_pushed) const final;
    static std::vector<probability_t> probs_from_beta(const int& s, const double& alpha);
};

#endif /* BUTTONWORLD_H_ */
