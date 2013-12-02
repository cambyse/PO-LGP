/*
 * util.cpp
 *
 *  Created on: Dec 12, 2012
 *      Author: robert
 */

#include "util.h"

#include <iostream>

using std::cout;
using std::endl;
using std::flush;
using std::vector;
using std::sort;

#include <QtCore>

#define DEBUG_STRING "util: "
#define DEBUG_LEVEL 0
#include "debug.h"

namespace util {

    bool arg_int(const QString& string_in, const int& n, int& i_arg_out) {
        QString arg = string_in.section(QRegExp("\\s+"),n,n);
        bool ok;
        i_arg_out = arg.toInt(&ok);
        return ok;
    }

    bool arg_double(const QString& string_in, const int& n, double& d_arg_out) {
        QString arg = string_in.section(QRegExp("\\s+"),n,n);
        bool ok;
        d_arg_out = arg.toDouble(&ok);
        return ok;
    }

    bool arg_string(const QString& string_in, const int& n, QString& s_arg_out) {
        s_arg_out = string_in.section(QRegExp("\\s+"),n,n);
        return true;
    }

    double approx_equal_tolerance() { return 1e-10; }

    AbstractIteratableSpace::Iterator::Iterator(ptr_t ptr): object(ptr) {}

    AbstractIteratableSpace::ptr_t AbstractIteratableSpace::Iterator::operator*() const {
        return object;
    }

    AbstractIteratableSpace::Iterator & AbstractIteratableSpace::Iterator::operator++() {
        object = object->next();
        return *this;
    }

    bool AbstractIteratableSpace::Iterator::operator!=(const Iterator& other) const {
        return *(this->object)!=*(other.object);
    }

    bool AbstractIteratableSpace::operator==(const AbstractIteratableSpace& other) const {
        return !(*this!=other);
    }

    const InvalidBase INVALID = InvalidBase(true);

    Range::Range(int first, int last, int increment):
        begin_idx(first),
        idx_increment(increment)
    {
        if(first<=last) {
            // range is forward (or zero)
            if(increment>0) {
                // increment is forward
                int offset = (last-first)%increment;
                end_idx=last-offset+increment;
            } else {
                // increment is backward (or zero)
                end_idx=first;
            }
        } else {
            // range is backward
            if(increment>=0) {
                // increment is forward (or zero)
                end_idx=first;
            } else {
                // increment is backward
                int offset = (first-last)%(-increment);
                end_idx=last+offset+increment;
            }
        }
    }

    Range::Range(int first, int last):
        Range(first,last,first<last?+1:-1)
    {}

    Range::Range(int last):
        Range(0,last-1,1)
    {}

    Range::RangeIt & Range::RangeIt::operator++() {
        idx+=idx_increment;
        return *this;
    }

    bool Range::RangeIt::operator== (const RangeIt& other) const {
        return other.idx==this->idx;
    }

    bool Range::RangeIt::operator!= (const RangeIt& other) const {
        return !(other==*this);
    }

    Range::RangeIt Range::begin() const {
        return RangeIt(begin_idx,idx_increment);
    }

    Range::RangeIt Range::end() const {
        return RangeIt(end_idx,idx_increment);
    }

    Range::RangeIt::RangeIt(int i, int incr):
        idx(i),
        idx_increment(incr)
    {}

    iRange::iRange(int n): Range(n-1,0,-1) {}

} // end namespace util
