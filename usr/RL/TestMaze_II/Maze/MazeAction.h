#ifndef MAZEACTION_H_
#define MAZEACTION_H_

#include "../AbstractAction.h"

class MazeAction: public AbstractAction {
public:
    enum class ACTION { UP, DOWN, LEFT, RIGHT, STAY, NONE };
    MazeAction(ACTION a = ACTION::NONE);
    MazeAction(const char * a);
    virtual Iterator begin() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractAction &other) const override;
    virtual bool operator<(const AbstractAction &other) const override;
    virtual const char * print() const override;
    inline virtual const std::string space_descriptor() const override { return "MazeAction"; }
    virtual inline ACTION get_action() const final { return action; }
    static MazeAction random_action() { return MazeAction((ACTION)(rand()%(int)ACTION::NONE)); }
protected:
    ACTION action;
    virtual void set_type(ACTION_TYPE t) override;
};

#endif /* MAZEACTION_H_ */
