#ifndef ABSTRACTINSTANCE_H_
#define ABSTRACTINSTANCE_H_

#include "../util/util.h"
#include "../util/debug.h"

class AbstractInstance: public util::AbstractIteratableSpace<AbstractInstance> {
public:
    AbstractInstance() = default;
    virtual ~AbstractInstance() = default;
    ABSTRACT_ITERATABLE_SPACE_BEGIN(AbstractInstance);
    virtual ptr_t next() const override;
    ABSTRACT_ITERATABLE_SPACE_NE(AbstractInstance);
    virtual bool operator!=(const AbstractInstance& other) const;
    ABSTRACT_ITERATABLE_SPACE_LT(AbstractInstance);
    virtual bool operator<(const AbstractInstance& other) const;
    virtual const std::string print() const;
    friend inline std::ostream& operator<<(std::ostream& out, const AbstractInstance& a) {
        return out << a.print();
    }
};

#include "../util/debug_exclude.h"

#endif /* ABSTRACTINSTANCE_H_ */
