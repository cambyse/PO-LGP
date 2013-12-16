#ifndef MINIMALACTION_H_
#define MINIMALACTION_H_

#include "../../AbstractAction.h"

#include "../../debug.h"

class MinimalAction: public AbstractAction {
public:
    enum ACTION { CHANGE, STAY, NONE } action;
    MinimalAction(ACTION a = ACTION::NONE) {
        action = a;
        set_type(ACTION_TYPE::MINIMAL);
    }
    virtual ~MinimalAction() = default;
    virtual Iterator begin() const override {
        return Iterator(new MinimalAction(CHANGE));
    }
    virtual ptr_t next() const override {
        if(action==CHANGE) {
            return ptr_t(new MinimalAction(STAY));
        } else {
            return ptr_t(new MinimalAction());
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
    virtual const char * print() const override {
        if(action==CHANGE) return std::string("MinimalAction(CHANGE)").c_str();
        if(action==STAY) return std::string("MinimalAction(STAY)").c_str();
        return std::string("MinimalAction(NONE)").c_str();
    }
    inline virtual const std::string space_descriptor() const override { return "MinimalAction"; }
};

#include "../../debug_exclude.h"

#endif /* MINIMALACTION_H_ */
