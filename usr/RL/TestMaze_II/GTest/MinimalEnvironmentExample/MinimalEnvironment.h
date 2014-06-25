#ifndef MINIMALENVIRONMENT_H_
#define MINIMALENVIRONMENT_H_

#include "../../PredictiveEnvironment.h"
#include "../../Representation/DoublyLinkedInstance.h"
#include "MinimalAction.h"
#include "MinimalObservation.h"
#include "MinimalReward.h"

#include "../util/ColorOutput.h"

class MinimalEnvironment: public PredictiveEnvironment {
public:
    MinimalEnvironment():
    PredictiveEnvironment(action_ptr_t(new MinimalAction(MinimalAction::ACTION::STAY)),
                          observation_ptr_t(new MinimalObservation(MinimalObservation::OBSERVATION::RED)),
                          reward_ptr_t(new MinimalReward(MinimalReward::REWARD::NO_REWARD)))
    {
        current_instance = DoublyLinkedInstance::create(action_space, observation_space, reward_space);
        action_space = action_ptr_t(new MinimalAction());
        observation_space = observation_ptr_t(new MinimalObservation());
        reward_space = reward_ptr_t(new MinimalReward());
    }
    virtual ~MinimalEnvironment() = default;

    virtual probability_t get_prediction(const_instance_ptr_t i, const action_ptr_t& a, const observation_ptr_t& o, const reward_ptr_t& r) const override {
        bool same_observation = i->observation==o;
        bool stay_action = a==MinimalAction(MinimalAction::ACTION::STAY);
        bool no_reward = r==MinimalReward(MinimalReward::REWARD::NO_REWARD);
        double HIGH = 10;
        double MEDIUM_HIGH = 4;
        double MEDIUM_LOW = 2;
        double LOW = 1;
        double p;
        if(same_observation) {
            if(stay_action) {
                if(no_reward) {
                    p = HIGH;
                } else {
                    p = MEDIUM_HIGH;
                }
            } else {
                if(no_reward) {
                    p = MEDIUM_LOW;
                } else {
                    p = LOW;
                }
            }
        } else {
            if(stay_action) {
                if(no_reward) {
                    p = MEDIUM_LOW;
                } else {
                    p = LOW;
                }
            } else {
                if(no_reward) {
                    p = HIGH;
                } else {
                    p = MEDIUM_HIGH;
                }
            }
        }
        return p/(HIGH + MEDIUM_HIGH + MEDIUM_LOW + LOW);
    }

    virtual void get_features(std::vector<f_ptr_t> & basis_features, FeatureLearner::LEARNER_TYPE type) const override {
        // all get current action
        if(type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE ||
           type==FeatureLearner::LEARNER_TYPE::HISTORY_ONLY ||
           type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION) {
            basis_features.push_back(ActionFeature::create(action_ptr_t(new MinimalAction(MinimalAction::ACTION::STAY)), 0));
            basis_features.push_back(ActionFeature::create(action_ptr_t(new MinimalAction(MinimalAction::ACTION::CHANGE)), 0));
        }
        // all but Linear-Q get current observation and reward
        if(type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE ||
           type==FeatureLearner::LEARNER_TYPE::HISTORY_ONLY) {
            basis_features.push_back(ObservationFeature::create(observation_ptr_t(new MinimalObservation(MinimalObservation::OBSERVATION::RED)), 0));
            basis_features.push_back(ObservationFeature::create(observation_ptr_t(new MinimalObservation(MinimalObservation::OBSERVATION::GREEN)), 0));
            basis_features.push_back(RewardFeature::create(reward_ptr_t(new MinimalReward(MinimalReward::REWARD::NO_REWARD)), 0));
            basis_features.push_back(RewardFeature::create(reward_ptr_t(new MinimalReward(MinimalReward::REWARD::SOME_REWARD)), 0));
        }
        // all but U-Tree get last data
        if(type==FeatureLearner::LEARNER_TYPE::FULL_PREDICTIVE ||
           type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION) {
            basis_features.push_back(ActionFeature::create(action_ptr_t(new MinimalAction(MinimalAction::ACTION::STAY)), -1));
            basis_features.push_back(ActionFeature::create(action_ptr_t(new MinimalAction(MinimalAction::ACTION::STAY)), -1));
            basis_features.push_back(ObservationFeature::create(observation_ptr_t(new MinimalObservation(MinimalObservation::OBSERVATION::RED)), -1));
            basis_features.push_back(ObservationFeature::create(observation_ptr_t(new MinimalObservation(MinimalObservation::OBSERVATION::GREEN)), -1));
            basis_features.push_back(RewardFeature::create(reward_ptr_t(new MinimalReward(MinimalReward::REWARD::NO_REWARD)), -1));
            basis_features.push_back(RewardFeature::create(reward_ptr_t(new MinimalReward(MinimalReward::REWARD::SOME_REWARD)), -1));
        }
        // Linear-Q also gets a unit feature
        if(type==FeatureLearner::LEARNER_TYPE::HISTORY_AND_ACTION) {
            basis_features.push_back(ConstFeature::create(1));
        }
    }

    virtual void get_spaces(action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r) const override {
        a = action_ptr_t(new MinimalAction());
        o = observation_ptr_t(new MinimalObservation());
        r = reward_ptr_t(new MinimalReward());
    }

    virtual void print_last_transition() const {
        using ColorOutput::reset_all;
        using ColorOutput::bold;
        using ColorOutput::fg_red;
        using ColorOutput::bg_red;
        using ColorOutput::bg_green;
        std::cout << reset_all() << "-------" << std::endl;
        if(current_instance->action==MinimalAction(MinimalAction::ACTION::STAY)) {
            std::cout << "   O  " << std::endl;
        } else {
            std::cout << "  <-> " << std::endl;
        }
        if(current_instance->observation==MinimalObservation(MinimalObservation::OBSERVATION::RED)) {
            std::cout << " |" << bg_green() << " " << reset_all() << "|" << bg_red() << bold() << "X" << reset_all() << "|";
        } else {
            std::cout << " |" << bg_green() << bold() << "X" << reset_all() << "|" << bg_red() << " " << reset_all() << "|";
        }
        if(current_instance->reward==MinimalReward(MinimalReward::REWARD::NO_REWARD)) {
            std::cout << " :-(";
        } else {
            std::cout << fg_red() << " :-)" << reset_all();
        }
        std::cout << std::endl;
        std::cout << "-------" << std::endl;
    }
};

#endif /* MINIMALENVIRONMENT_H_ */
