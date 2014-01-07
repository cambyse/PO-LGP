#ifndef CHESEMAZEACTION_H_
#define CHESEMAZEACTION_H_

#include "../AbstractAction.h"

class CheseMazeAction: public AbstractAction {
public:
    enum class ACTION { NORTH, SOUTH, WEST, EAST, END };
    CheseMazeAction(ACTION a = ACTION::NORTH);
    CheseMazeAction(const char * a);
    ABSTRACT_ITERATABLE_SPACE_BEGIN(CheseMazeAction);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractAction &other) const override;
    virtual bool operator<(const AbstractAction &other) const override;
    virtual const std::string print() const override;
    virtual inline ACTION get_action() const final { return action; }
    static CheseMazeAction random_action() { return CheseMazeAction((ACTION)(rand()%(int)ACTION::END)); }
protected:
    ACTION action;
    virtual void set_type(ACTION_TYPE t) override;
};

#endif /* CHESEMAZEACTION_H_ */
