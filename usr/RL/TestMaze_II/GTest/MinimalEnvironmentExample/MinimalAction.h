#ifndef MINIMALACTION_H_
#define MINIMALACTION_H_

#include "../../Representation/AbstractAction.h"

class MinimalAction: public AbstractAction {
public:
    enum ACTION { CHANGE, STAY } action;
    MinimalAction(ACTION a = ACTION::CHANGE);
    virtual ~MinimalAction() override = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(MinimalAction);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractAction &other) const override;
    virtual bool operator<(const AbstractAction &other) const override;
    virtual const std::string print() const override;
};

#endif /* MINIMALACTION_H_ */
