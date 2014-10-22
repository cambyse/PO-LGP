#ifndef MAZEACTION_H_
#define MAZEACTION_H_

#include "../Representation/AbstractAction.h"

class MazeAction: public AbstractAction {
public:
    enum class ACTION { UP, DOWN, LEFT, RIGHT, STAY, END };
    MazeAction(ACTION a = ACTION::UP);
    MazeAction(const char * a);
    virtual ~MazeAction() override {}
    ABSTRACT_ITERATABLE_SPACE_BEGIN(MazeAction);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractAction &other) const override;
    virtual bool operator<(const AbstractAction &other) const override;
    virtual const std::string print() const override;
    virtual inline ACTION get_action() const final { return action; }
    static MazeAction random_action() { return MazeAction((ACTION)(rand()%(int)ACTION::END)); }
protected:
    ACTION action;
    virtual void set_type(ACTION_TYPE t) override;
};

#endif /* MAZEACTION_H_ */
