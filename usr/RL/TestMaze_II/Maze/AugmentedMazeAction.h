#ifndef AUGMENTEDMAZEACTION_H_
#define AUGMENTEDMAZEACTION_H_

#include "MazeAction.h"

class AugmentedMazeAction: public MazeAction {
public:
    enum class TAG { TAG_0, TAG_1, TAG_2, END } tag;
    AugmentedMazeAction();
    AugmentedMazeAction(ACTION a, TAG t = TAG::TAG_0);
    ABSTRACT_ITERATABLE_SPACE_BEGIN(AugmentedMazeAction);
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractAction &other) const override;
    virtual bool operator<(const AbstractAction &other) const override;
    virtual const std::string print() const override;
};

#endif /* AUGMENTEDMAZEACTION_H_ */
