#ifndef OBSERVATION_H_
#define OBSERVATION_H_

#include "../util.h"

#include <ostream>

/** \brief Observation objects. */
class Observation: public util::NumericTypeWrapper<Observation, int>  {
public:
    static const value_t min_observation;
    static const value_t max_observation;
    static const unsigned long observation_n;
    Observation(value_t val = min_observation);
    friend std::ostream& operator<<(std::ostream &out, const Observation& s);
    static Observation random_observation();
    unsigned long index() const { return *this - min_observation; }
private:
    int add_width() const;
};

/** \brief ObservationIt objects.
 *
 * ObservationIt are iterators over Observation objects. They are derived from the Observation
 * class and can hence directly be used like Observation object without dereferencing
 * them. */
class ObservationIt: public Observation, public util::InvalidAdapter<ObservationIt> {

public:

    // for compatibility with for( ... : ... ) constructs
    struct All {
        static ObservationIt begin() { return ObservationIt::first(); }
        static ObservationIt end() { return ObservationIt(); }
    };
    static const All all;

    // Make operator resolution unambiguous.
    // Try using the Invalid adapter first.
    using util::InvalidAdapter<ObservationIt>::operator!=;
    using util::NumericTypeWrapper<Observation, value_t>::operator!=;
    using util::InvalidAdapter<ObservationIt>::operator==;
    using util::NumericTypeWrapper<Observation, value_t>::operator==;

    ObservationIt();
    ObservationIt(const Observation& s);
    Observation operator*() { return *this; }
    ObservationIt & operator++();
    ObservationIt & operator--();
    ObservationIt & operator+=(const int& c);
    ObservationIt & operator-=(const int& c);
    ObservationIt operator+(const int& c) const { return ObservationIt(*this)+=c; }
    ObservationIt operator-(const int& c) const { return ObservationIt(*this)-=c; }

    static const ObservationIt first();
    static const ObservationIt last();

private:
    void check_for_invalid();
};

#endif // OBSERVATION_H_
