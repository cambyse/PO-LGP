#ifndef AUGMENTEDMAZEACTION_H_
#define AUGMENTEDMAZEACTION_H_

#include "MazeAction.h"

class AugmentedMazeAction: public MazeAction {
public:
    enum class TAG { TAG_0, TAG_1, TAG_2, NONE } tag;
    AugmentedMazeAction(ACTION a = ACTION::NONE, TAG t = TAG::NONE);
    virtual Iterator begin() const override;
    virtual Iterator end() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractAction &other) const override;
    virtual std::string print() const override;
};

#endif /* AUGMENTEDMAZEACTION_H_ */
