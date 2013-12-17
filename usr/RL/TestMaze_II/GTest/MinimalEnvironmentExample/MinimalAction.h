#ifndef MINIMALACTION_H_
#define MINIMALACTION_H_

#include "../../AbstractAction.h"

#include "../../debug.h"

class MinimalAction: public AbstractAction {
public:
    enum ACTION { CHANGE, STAY } action;
    MinimalAction(ACTION a = ACTION::CHANGE) {
        action = a;
        set_type(ACTION_TYPE::MINIMAL);
    }
    virtual ~MinimalAction() = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(MinimalAction);
    virtual ptr_t next() const override {
        if(action==CHANGE) {
            return ptr_t(new MinimalAction(STAY));
        } else {
            return ptr_t(new AbstractAction());
        }
    }
    virtual bool operator!=(const AbstractAction &other) const override {
        if(this->get_type()!=other.get_type()) {
            return true;
        } else {
            auto minimal_action = dynamic_cast<const MinimalAction *>(&other);
            if(minimal_action==nullptr) {
                DEBUG_ERROR("Dynamic cast failed");
                return true;
            } else {
                return this->action!=minimal_action->action;
            }
        }
    }
    virtual bool operator<(const AbstractAction &other) const override {
        if(this->get_type()<other.get_type()) {
            return true;
        } else {
            auto minimal_action = dynamic_cast<const MinimalAction *>(&other);
            if(minimal_action==nullptr) {
                DEBUG_ERROR("Dynamic cast failed");
                return true;
            } else {
                return this->action<minimal_action->action;
            }
        }
    }
    virtual const std::string print() const override {
        switch(action) {
        case ACTION::CHANGE:
            return std::string("MinimalAction(CHANGE)");
        case ACTION::STAY:
            return std::string("MinimalAction(STAY)");
        default:
            DEBUG_DEAD_LINE;
            return std::string("MinimalAction(INVALID)");
        }
    }
};

#include "../../debug_exclude.h"

#endif /* MINIMALACTION_H_ */
