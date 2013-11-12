#ifndef AUGMENTEDOBSERVATION_H_
#define AUGMENTEDOBSERVATION_H_

#include "../util.h"

#include <ostream>

/** \brief AugmentedObservation objects. */
class AugmentedObservation: public util::NumericTypeWrapper<AugmentedObservation, int>  {
public:
    static const value_t min_observation;
    static const value_t max_observation;
    static const unsigned long observation_n;
    AugmentedObservation(value_t val = min_observation);
    friend std::ostream& operator<<(std::ostream &out, const AugmentedObservation& s);
    static AugmentedObservation random_observation();
    unsigned long index() const { return *this - min_observation; }
private:
    int add_width() const;
};

/** \brief AugmentedObservationIt objects.
 *
 * AugmentedObservationIt are iterators over AugmentedObservation objects. They are derived from the AugmentedObservation
 * class and can hence directly be used like AugmentedObservation object without dereferencing
 * them. */
class AugmentedObservationIt: public AugmentedObservation, public util::InvalidAdapter<AugmentedObservationIt> {

public:

    // for compatibility with for( ... : ... ) constructs
    struct All {
        static AugmentedObservationIt begin() { return AugmentedObservationIt::first(); }
        static AugmentedObservationIt end() { return AugmentedObservationIt(); }
    };
    static const All all;

    // Make operator resolution unambiguous.
    // Try using the Invalid adapter first.
    using util::InvalidAdapter<AugmentedObservationIt>::operator!=;
    using util::NumericTypeWrapper<AugmentedObservation, value_t>::operator!=;
    using util::InvalidAdapter<AugmentedObservationIt>::operator==;
    using util::NumericTypeWrapper<AugmentedObservation, value_t>::operator==;

    AugmentedObservationIt();
    AugmentedObservationIt(const AugmentedObservation& s);
    AugmentedObservation operator*() { return *this; }
    AugmentedObservationIt & operator++();
    AugmentedObservationIt & operator--();
    AugmentedObservationIt & operator+=(const int& c);
    AugmentedObservationIt & operator-=(const int& c);
    AugmentedObservationIt operator+(const int& c) const { return AugmentedObservationIt(*this)+=c; }
    AugmentedObservationIt operator-(const int& c) const { return AugmentedObservationIt(*this)-=c; }

    static const AugmentedObservationIt first();
    static const AugmentedObservationIt last();

private:
    void check_for_invalid();
};

#endif // AUGMENTEDOBSERVATION_H_
