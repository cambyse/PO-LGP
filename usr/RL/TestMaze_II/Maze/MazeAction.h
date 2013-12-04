#ifndef MAZEACTION_H_
#define MAZEACTION_H_

#include "../AbstractAction.h"

class MazeAction: public AbstractAction {
public:
    enum class ACTION { UP, DOWN, LEFT, RIGHT, STAY, NONE } action;
    MazeAction(ACTION a = ACTION::NONE);
    virtual Iterator begin() const override;
    virtual Iterator end() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractAction &other) const override;
    virtual std::string print() const override;
protected:
    virtual void set_type(ACTION_TYPE t) override;
};

#endif /* MAZEACTION_H_ */
