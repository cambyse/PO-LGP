#ifndef ABSTRACTACTION_H_
#define ABSTRACTACTION_H_

#include "util.h"

class AbstractAction: public util::AbstractIteratableSpace {
public:
    enum class ACTION_TYPE { NONE, MAZE_ACTION, AUGMENTED_MAZE_ACTION };
    AbstractAction(ACTION_TYPE t = ACTION_TYPE::NONE);
    virtual Iterator begin() const override;
    virtual Iterator end() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractIteratableSpace& other) const override;
    virtual bool operator!=(const AbstractAction& other) const;
    virtual std::string print() const override;
    friend std::ostream& operator<<(std::ostream& out, const AbstractAction& a);
    ACTION_TYPE get_type() const;
protected:
    virtual void set_type(ACTION_TYPE t);
private:
    ACTION_TYPE action_type;
};

#endif /* ABSTRACTACTION_H_ */
