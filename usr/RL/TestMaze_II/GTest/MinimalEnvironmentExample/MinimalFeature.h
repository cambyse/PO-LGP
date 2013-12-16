#ifndef MINIMALFEATURE_H_
#define MINIMALFEATURE_H_

#include "../Config.h"
#include "MinimalAction.h"
#include "MinimalObservation.h"
#include "MinimalReward.h"

class MinimalFeature: public BasisFeature {
public:
    USE_CONFIG_TYPEDEFS;
    enum TYPE { CURRENT_ACTION, CURRENT_OBSERVATION, CURRENT_REWARD,
                LAST_ACTION, LAST_OBSERVATION, LAST_REWARD};
private:
    MinimalFeature(
        const TYPE& t,
        const action_ptr_t& a,
        const observation_ptr_t& o,
        const reward_ptr_t& r
        ):
    type(t), action(a), observation(o), reward(r)
    {
        feature_type = MINIMAL;
    }
public:
    virtual ~MinimalFeature() = default;
    static const_feature_ptr_t create(
        const TYPE& t,
        const action_ptr_t& a,
        const observation_ptr_t& o,
        const reward_ptr_t& r
        ) {
        MinimalFeature * new_feature = new MinimalFeature(t,a,o,r);
        const_feature_ptr_t return_ptr(new_feature);
        new_feature->set_this_ptr(return_ptr);
        return return_ptr;
    }
    virtual feature_return_value evaluate(const_instanceIt_t insIt) const {
        switch(type) {
        case CURRENT_ACTION:
            if( insIt!=util::INVALID && insIt->action==action) {
                return return_function(1);
            }
            break;
        case CURRENT_OBSERVATION:
            if( insIt!=util::INVALID && insIt->observation==observation) {
                return return_function(1);
            }
            break;
        case CURRENT_REWARD:
            if( insIt!=util::INVALID && insIt->reward==reward) {
                return return_function(1);
            }
            break;
        case LAST_ACTION:
            if((insIt-=1)!=util::INVALID  && insIt->action==action) {
                return return_function(1);
            }
            break;
        case LAST_OBSERVATION:
            if((insIt-=1)!=util::INVALID  && insIt->observation==observation) {
                return return_function(1);
            }
            break;
        case LAST_REWARD:
            if((insIt-=1)!=util::INVALID  && insIt->reward==reward) {
                return return_function(1);
            }
            break;
        }
        return return_function(0);
    }
    virtual std::string identifier() const {
        QString id;
        switch(type) {
        case CURRENT_ACTION:
            id = "CURRENT_ACTION: %1";
            id.arg(action->print());
            break;
        case CURRENT_OBSERVATION:
            id = "CURRENT_OBSERVATION: %1";
            id.arg(observation->print());
            break;
        case CURRENT_REWARD:
            id = "CURRENT_REWARD: %1";
            id.arg(reward->print());
            break;
        case LAST_ACTION:
            id = "LAST_ACTION: %1";
            id.arg(action->print());
            break;
        case LAST_OBSERVATION:
            id = "LAST_OBSERVATION: %1";
            id.arg(observation->print());
            break;
        case LAST_REWARD:
            id = "LAST_REWARD: %1";
            id.arg(reward->print());
            break;
        }
        return id.toStdString()+Feature::identifier();
    }
    virtual bool operator==(const Feature& other) const override {
        auto * ptr = dynamic_cast<const MinimalFeature *>(&other);
        return (
            ptr!=nullptr &&
            action==ptr->action &&
            observation==ptr->observation &&
            reward==ptr->reward
            );
    }
protected:
    TYPE type;
    action_ptr_t action;
    observation_ptr_t observation;
    reward_ptr_t reward;
};

#endif /* MINIMALFEATURE_H_ */
