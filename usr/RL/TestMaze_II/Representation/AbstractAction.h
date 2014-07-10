#ifndef ABSTRACTACTION_H_
#define ABSTRACTACTION_H_

#include "../util/util.h"
#include "../util/debug.h"

/** \brief Base class for all actions. */
class AbstractAction: public util::AbstractIteratableSpace<AbstractAction> {
public:
    enum class ACTION_TYPE { NONE, MINIMAL, MAZE_ACTION, AUGMENTED_MAZE_ACTION, CHEESE_MAZE_ACTION };
    AbstractAction(ACTION_TYPE t = ACTION_TYPE::NONE);
    virtual ~AbstractAction() {}
    ABSTRACT_ITERATABLE_SPACE_BEGIN(AbstractAction);
    virtual ptr_t next() const override;
    ABSTRACT_ITERATABLE_SPACE_NE(AbstractAction);
    virtual bool operator!=(const AbstractAction& other) const;
    ABSTRACT_ITERATABLE_SPACE_LT(AbstractAction);
    virtual bool operator<(const AbstractAction& other) const;
    virtual const std::string print() const;
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractAction& a) {
        return out << a.print();
    }
    virtual ACTION_TYPE get_type() const;
protected:
    bool print_short_name = true;
    virtual void set_type(ACTION_TYPE t);
private:
    ACTION_TYPE action_type;
};

#include "../util/debug_exclude.h"

#endif /* ABSTRACTACTION_H_ */
