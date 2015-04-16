#ifndef BUTTONWORLD_H_
#define BUTTONWORLD_H_

#include <environment/PredictiveEnvironment.h>
#include <environment/Visualizer.h>

#include <QGraphicsSvgItem>

#include <config/Config.h>
#include <util/util.h>
#include <representation/Feature.h>
#include "ButtonAction.h"
#include <representation/UniqueObservation.h>
#include <representation/ListedReward.h>

class ButtonWorld: public PredictiveEnvironment, public Visualizer  {

    //----typedefs/classes----//
public:
    USE_CONFIG_TYPEDEFS;
    typedef ButtonAction action_t;
    typedef UniqueObservation observation_t;
    typedef ListedReward reward_t;

    //----members----//
public:
    const int size;
    static const bool use_factored_action_features;
protected:
    std::vector<probability_t> prob_array;
    std::vector<QGraphicsRectItem*> button_array;

    //----methods----//
public:
    virtual ~ButtonWorld() override = default;
    virtual void get_features(f_set_t & basis_features,
                              FeatureLearner::LEARNER_TYPE type) const override;
    static void construct_factored_action_features(f_set_t & basis_features,
                                                   action_ptr_t action,
                                                   int delay);
    double get_p_sum() const;
protected:
    ButtonWorld(int s = 1, std::vector<probability_t> p = std::vector<probability_t>());
    static std::vector<probability_t> probs_from_beta(const int& s, const double& alpha);
};

#endif /* BUTTONWORLD_H_ */
