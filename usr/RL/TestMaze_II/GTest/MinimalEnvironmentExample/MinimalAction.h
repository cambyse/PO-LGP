#ifndef MINIMALACTION_H_
#define MINIMALACTION_H_

#include "../../AbstractAction.h"

class MinimalAction: public AbstractAction {
public:
    enum ACTION { ONE, TWO, NONE } action;
    MazeAction(ACTION a = NONE) {
        action = a;
        set_type(ACTION_TYPE::MINIMAL);
    }
    virtual ~MinimalAction() = default;
    virtual Iterator begin() const override {
        return Iterator(new MazeAction(ONE));
    }
    virtual ptr_t next() const override {
        if(action==ONE) return ptr_t(new MinimalAction(TWO));
        return end();
    }
    virtual bool operator!=(const AbstractAction &other) const override {
        return action!=other.action;
    }
    virtual bool operator<(const AbstractAction &other) const override {
        return action<other.action;
    }
    virtual const char * print() const override {
        if(action==ONE) return std::string("MinimalAction(ONE)").c_str();
        if(action==TWO) return std::string("MinimalAction(TWO)").c_str();
        return std::string("MinimalAction(NONE)").c_str();
    }
    inline virtual const std::string space_descriptor() const override { return "MinimalAction"; }
};

#endif /* MINIMALACTION_H_ */
