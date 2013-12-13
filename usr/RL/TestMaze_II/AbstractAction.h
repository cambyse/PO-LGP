#ifndef ABSTRACTACTION_H_
#define ABSTRACTACTION_H_

#include "util.h"

class AbstractAction: public util::AbstractIteratableSpace<AbstractAction> {
public:
    enum class ACTION_TYPE { NONE, MINIMAL, MAZE_ACTION, AUGMENTED_MAZE_ACTION };
    AbstractAction(ACTION_TYPE t = ACTION_TYPE::NONE);
    virtual Iterator begin() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractIteratableSpace& other) const override;
    virtual bool operator!=(const AbstractAction& other) const;
    virtual bool operator<(const AbstractIteratableSpace& other) const override;
    virtual bool operator<(const AbstractAction& other) const;
    virtual const char * print() const;
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractAction& a) {
        return out << a.print();
    }
    virtual ACTION_TYPE get_type() const;
    inline virtual const std::string space_descriptor() const override { return "AbstractAction"; }
protected:
    virtual void set_type(ACTION_TYPE t);
private:
    ACTION_TYPE action_type;
};

#endif /* ABSTRACTACTION_H_ */
