#ifndef CHEESEMAZEACTION_H_
#define CHEESEMAZEACTION_H_

#include <representation/AbstractAction.h>

class CheeseMazeAction: public AbstractAction {
public:
    enum class ACTION { NORTH, SOUTH, WEST, EAST, END };
    CheeseMazeAction(ACTION a = ACTION::NORTH);
    CheeseMazeAction(const std::string a);
    virtual ~CheeseMazeAction() override {}
    ABSTRACT_ITERATABLE_SPACE_BEGIN(CheeseMazeAction);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractAction &other) const override;
    virtual bool operator<(const AbstractAction &other) const override;
    virtual const std::string print() const override;
    virtual inline ACTION get_action() const final { return action; }
    static CheeseMazeAction random_action() { return CheeseMazeAction((ACTION)(rand()%(int)ACTION::END)); }
protected:
    ACTION action;
    virtual void set_type(ACTION_TYPE t) override;
};

#endif /* CHEESEMAZEACTION_H_ */
