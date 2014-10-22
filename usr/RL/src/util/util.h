/**
 * @file
 * \brief This file contains an unsorted set of helper functions, templates etc.
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <math.h>
#include <float.h> // for DBL_MAX
#include <limits> // e.g. std::numeric_limits<double>::max()
#include <QString>
#include <vector>
#include <sstream>
#include <algorithm> // for std::max
#include <memory> // for shared_ptr

#define DEBUG_LEVEL 0
#define DEBUG_STRING "util: "
#include "debug.h"

//========================================================//
//                       Macros                           //
//========================================================//

/** \brief Shortcut to replace for loop where the index does not play a role. */
#define repeat(n) for(unsigned long int repeat_index=0; repeat_index<(unsigned long int)n; ++repeat_index)

/** \brief Simplify comparison in hierarchies of abstract classes. */
#define COMPARE_ABSTRACT_TYPE_AND_CAST(comp,type_getter,type_for_cast)  \
    auto ptr = dynamic_cast<type_for_cast>(&other);                     \
    if(this->type_getter()!=other.type_getter()) {                      \
        return this->type_getter() comp other.type_getter();            \
    }                                                                   \
    if(ptr==nullptr) {                                                  \
        DEBUG_ERROR("Dynamic cast failed");                             \
        return true;                                                    \
    }

/** All utility functions etc are included in the util namespace. */
namespace util {

    //========================================================//
    //              Function Declarations                     //
    //========================================================//

    /** \brief Interprete n-th argument of string as integer.
     *
     * Returns true on success and false otherwise. */
    bool arg_int(const QString& string, const int& n, int& i);

    /** \brief Interprete n-th argument of string as double.
     *
     *  Returns true on success and false otherwise. */
    bool arg_double(const QString& string, const int& n, double& d);

    /** \brief Interprete n-th argument of string as string.
     *
     *  Returns true on success and false otherwise. */
    bool arg_string(const QString& string, const int& n, QString& s);

    /** \brief Tolerance for approximate comparison. */
    double approx_equal_tolerance();

    /** \brief Get a std::string if operator<< is defined. */
    template<class T>
    std::string string_from_ostream(const T& t) {
        std::stringstream s;
        s << t;
        return s.str();
    }

    //========================================================//
    //                      Classes                           //
    //========================================================//

    /** Grabs a stream like std::cout. The stream is redirected to as
     * std::stringstream and reset on destruction of the GrabStream object. The
     * content can be accessed using the get_text() method.*/
    class GrabStream {
    public:
    GrabStream(std::ostream & s = std::cout):
        stream(s),
            buffer(),
            old(stream.rdbuf(buffer.rdbuf()))
            {}
        ~GrabStream() {
            stream.rdbuf(old);
        }
        std::string get_text() { return buffer.str(); }
    private:
        /** The stream that was grabbed. */
        std::ostream & stream;
        /** The std::stringstream buffer the stream is redirected to. */
        std::stringstream buffer;
        /** The old buffer the stream is reset to on destruction of this
         * object. */
        std::streambuf * old;
    };

    /** \brief Comparison of pointers via their pointed-to objects. */
    template<class A, class B = A>
        class deref_less {
    public:
        bool operator()(const A& a, const B& b) { return *a<*b; }
    };
    template<>
        template<class P>
        class deref_less<std::weak_ptr<P> > {
    public:
        bool operator()(const std::weak_ptr<P>& a, const std::weak_ptr<P>& b) {
            if(a.expired() || b.expired()) {
                DEBUG_ERROR("Pointer expired");
                return false;
            } else {
                return *(a.lock())<*(b.lock());
            }
        }
    };

    /** \brief Base class for polymorphic iteratable spaces.
     *
     * OUT OF DATE!!! An iteratable space consists of a countable set of
     * constant objects like actions in a maze (up, down, left, right, stay) or
     * the like. To define an iteratable space you can inherit from this class
     * and override the begin() end() next() and operator!=() methods. You can
     * iterate over any AbstractIteratableSpace as @code
     * for(YourIteratableSpace::ptr_t item : YourIteratableSpace()) {

         ...do things that can be done on any abstract space...

         // do a cast if you need the functionality
         // of your specific implemenation
         const YourIteratableSpace * your_item;
         your_item = dynamic_cast<const YourIteratableSpace *>(item.get());
         if(your_item!=nullptr) {

             ...do things that can only be done on your specific implementation...

         }
     }
     @endcode
     * You can use
     @code
     for(auto item : YourIteratableSpace()) { ... }
     @endcode
     * to keep the code brief. */
    template <class DerivedSpace>
    class AbstractIteratableSpace {
    public:

        class Iterator; // forward declaration for use in PointerType

        /** \brief Pointer class used as return type.
         *
         * The class uses std::shared_ptr as underlying type but defines
         * comparison operators based on the object pointed to (not the
         * adress).*/
        class PointerType {
        public:
        PointerType(): ptr(new const DerivedSpace()) {}
        PointerType(const DerivedSpace * d): ptr(d) {}
        PointerType(std::shared_ptr<const DerivedSpace> d): ptr(d) {}
            PointerType(const PointerType&) = default;
            virtual ~PointerType() final = default;
            virtual const DerivedSpace & operator*() const final {
                return ptr.operator*();
            }
            virtual const DerivedSpace * operator->() const final {
                return ptr.operator->();
            }
            virtual Iterator begin() const final {
                return ptr->begin();
            }
            virtual Iterator end() const final {
                return ptr->end();
            }
            virtual bool operator!=(const DerivedSpace& other) const final {
                return *(this->ptr)!=other;
            }
            virtual bool operator!=(const PointerType& other) const final {
                return *(this->ptr)!=*(other.ptr);
            }
            virtual bool operator!=(const nullptr_t) const final {
                return *(this->ptr)!=DerivedSpace();
            }
            virtual bool operator==(const DerivedSpace& other) const final {
                return !(*this!=other);
            }
            virtual bool operator==(const PointerType& other) const final {
                return !(*this!=other);
            }
            virtual bool operator==(const nullptr_t) const final {
                return !(this->operator!=(nullptr));
            }
            virtual bool operator<(const PointerType& other) const final {
                return *(this->ptr)<*(other.ptr);
            }
            friend inline std::ostream& operator<<(std::ostream& out, const PointerType& ptr) {
                return out << *ptr;
            }
            template < class T > std::shared_ptr<const T> get_derived(bool report_error = true) const {
                std::shared_ptr<const T> ret_ptr = std::dynamic_pointer_cast<const T>(ptr);
                if(report_error && ret_ptr == std::shared_ptr<const T>()) {
                    DEBUG_ERROR("Cast failed");
                }
                return ret_ptr;
            }
        private:
            std::shared_ptr<const DerivedSpace> ptr;
        };

        typedef PointerType ptr_t;

        /** \brief Iterator class.
         *
         * Dereferences to a pointer of the current object and calls the next()
         * function of the object to increment.*/
        class Iterator {

        public:

            Iterator(ptr_t ptr): object(ptr) {}

            Iterator(const DerivedSpace * ptr): object(ptr_t(ptr)) {}

            /** \brief Default descructor. */
            virtual ~Iterator() final = default;

            /** \brief Dereference operator returns pointer. */
            virtual ptr_t operator*() const final {
                return object;
            }

            /** Member access operator. */
            virtual const ptr_t * operator->() const final {
                return &object;
            }

            /** \brief Increment operator.
             *
             * Calls the next() function of the underlying object. */
            virtual Iterator & operator++() final {
                object = object->next();
                return *this;
            }

            /** \brief Inequality operator.
             *
             * Checks inequality based on the underlying objects. */
            virtual bool operator!=(const Iterator& other) const final {
                return *(this->object)!=*(other.object);
            }

        private:

            /** \brief The current object. */
            ptr_t object;
        };

        AbstractIteratableSpace() = default;

        virtual ~AbstractIteratableSpace() = default;

        /** \brief Returns Iterator to first object of the space. */
        virtual Iterator begin() const = 0;

        /** \brief Macro that should be used in derived classes to define the
         * begin function, so that it returns a default constructed object of
         * the derived class. */
#define ABSTRACT_ITERATABLE_SPACE_BEGIN(C)              \
        virtual Iterator begin() const override {       \
            return Iterator(ptr_t(new C()));            \
        }

        /** \brief Returns an invalid Iterator to stop iteration.
         *
         * A derived class must somehow implement an 'invalid' object. The
         * next() function must return a pointer to such an object if called on
         * the last element of the space and the end() function must return such
         * an object. */
        virtual Iterator end() const final {
            return Iterator(ptr_t(new const DerivedSpace()));
        }

        /** \brief Return pointer to next element in the space.
         *
         * When called on the last element in the space is should return an
         * invalid object (cf. end()). */
        virtual ptr_t next() const = 0;

        /** \brief Inequality operator.
         *
         * Used by Iterator class to determine stopping criterion. */
        virtual bool operator!=(const AbstractIteratableSpace& other) const = 0;

        virtual bool operator!=(const PointerType& other) const final {
            return *this!=*other;
        }

        /** \brief Macro that can be used in abstract derived classes for
         * convenience. */
#define ABSTRACT_ITERATABLE_SPACE_NE(C)                                 \
        virtual bool operator!=(const AbstractIteratableSpace& other) const override { \
            const C * derived_class = dynamic_cast<const C *>(&other);  \
            if(derived_class==nullptr) {                                \
                DEBUG_ERROR("Dynamic cast failed");                     \
                return true;                                            \
            } else {                                                    \
                return *this!=*derived_class;                           \
            }                                                           \
        }

        /** \brief Equality operator.
         *
         * To define equality simply define the inequality operator
         * accordingly. To ensure logical consistency this operator cannot be
         * overridden, instead it returns the negation of the inequality
         * operator. */
        virtual bool operator==(const AbstractIteratableSpace& other) const final {
            return !(*this!=other);
        }

        virtual bool operator==(const PointerType& other) const final {
            return !(*this!=other);
        }

        /** \brief operator< */
        virtual bool operator<(const AbstractIteratableSpace& other) const = 0;

        /** \brief Macro that can be used in abstract derived classes for
         * convenience. */
#define ABSTRACT_ITERATABLE_SPACE_LT(C)                                 \
        virtual bool operator<(const AbstractIteratableSpace& other) const override { \
            const C * derived_class = dynamic_cast<const C *>(&other);  \
            if(derived_class==nullptr) {                                \
                DEBUG_ERROR("Dynamic cast failed");                     \
                return true;                                            \
            } else {                                                    \
                return *this<*derived_class;                            \
            }                                                           \
        }

        virtual unsigned long int space_size() const {
            unsigned long int size = 0;
            for(ptr_t element : *this) {
                ++size;
            }
            return size;
        }

        virtual ptr_t random_element() const {
            Iterator elem = this->begin();
            int counter = rand()%this->space_size();
            while(counter>0) {
                ++elem;
                --counter;
            }
            return *elem;
        }

        virtual long int index() const {
            DEBUG_WARNING("Using inefficient method to compute index");
            unsigned long int idx = -1;
            for(ptr_t element : *this) {
                ++idx;
                if(*this==element) {
                    break;
                }
            }
            return idx;
        }
    };

    /** \brief Base class to make a derived class assign-compatible with a type.
     *
     * Overloads assignment and cast operators so that the class can be assigned
     * like being of type T.
     *
     * @code
     * NewClass: public AssignableTypeWrapper<NewClass,int> {
     *     NewClass(int val): AssignableTypeWrapper<NewClass,int>(val) {}
     * }
     * @endcode */
    template <class C, class T>
        class AssignableTypeWrapper {
    public:
        typedef T value_t;
    AssignableTypeWrapper(const T& val): value(val) { }
        operator T() const { return value; }
        C& operator=(const T &rhs) {
            this->value=rhs;
            return (*this);
        }
    protected:
        T value;
    };

    /** \brief Base class to provide a derived class with comparison operator.
     *
     * Inherits from util::AssignableTypeWrapper and additionally overloads
     * comparison operators so that objects of the class can be compared.
     *
     * @code
     * NewClass: public ComparableTypeWrapper<NewClass,int> {
     *     NewClass(int val): ComparableTypeWrapper<NewClass,int>(val) {}
     * }
     * @endcode */
    template <class C, class T>
        class ComparableTypeWrapper: public AssignableTypeWrapper<C,T> {
    public:
    ComparableTypeWrapper(T val): AssignableTypeWrapper<C,T>(val) { }
        bool operator==(const T &other) const { return this->value==other; }
        bool operator!=(const T &other) const { return this->value!=other; }
        bool operator<(const T &other) const { return this->value<other; }
        bool operator>(const T &other) const { return this->value>other; }
        bool operator<=(const T &other) const { return this->value<=other; }
        bool operator>=(const T &other) const { return this->value>=other; }
    };

    /** \brief Base class to provide a derived class with standard arithmetic
     * operators.
     *
     * Inherits from util::ComparableTypeWrapper and additionally overloads
     * arithmetic operators so that objects of the class can be subtracted,
     * multiplied etc.
     *
     * @code
     * NewClass: public NumericTypeWrapper<NewClass,int> {
     *     NewClass(int val): NumericTypeWrapper<NewClass,int>(val) {}
     * }
     * @endcode */
    template <class C, class T>
        class NumericTypeWrapper: public ComparableTypeWrapper<C,T> {
    public:
    NumericTypeWrapper(T val): ComparableTypeWrapper<C,T>(val) { }
        T & operator+=(const T &rhs) { return this->value+=rhs; }
        T & operator-=(const T &rhs) { return this->value-=rhs; }
        T & operator*=(const T &rhs) { return this->value*=rhs; }
        T & operator/=(const T &rhs) { return this->value/=rhs; }
    };

    /** \brief Base for objects that can be invalid.
     *
     * The globale constant util::INVALID is of this type. */
    class InvalidBase {
    public:
        /** \brief Default constructor constructs an invalid object. */
    InvalidBase(const bool& b = true): invalid(b) {}
        /** \brief Returns whether the object is invalid. */
        bool is_invalid() const { return invalid; }
        bool operator!=(const InvalidBase& other) const { return invalid!=other.invalid; }
        bool operator==(const InvalidBase& other) const { return !(*this!=other); }
    protected:
        /** \brief Holds whether the object is invalid. */
        bool invalid;
        /** \brief Invalidate this object.
         *
         * Sets this->invalid to true but keeps the rest untouched. */
        void invalidate() {
            this->invalid=true;
        }
    };

    /** \brief Base class to make a derived class comparable and assignable with
     * the globale util::INVALID object of this class.
     *
     * @code
     * NewClass: public InvalidAdapter<NewClass> {
     *     // Default constructor constructs an invalid object
     *     // if not specified differently.
     *     NewClass(): InvalidAdapter<NewClass>(true) {} // <-- redundant
     *     NewClass(): InvalidAdapter<NewClass>(false) {} // <-- non-invalid object
     * }
     * @endcode */
    template <class C>
        class InvalidAdapter: public InvalidBase {
    protected:
        /** \brief Default constructor constructs an invalid object. */
    InvalidAdapter(const bool& b = true): InvalidBase(b) {}
    public:
        /** \brief InvalidAdapter objects can be compared to InvalidBase objects.
         *
         * Returns true if both objects are valid/invalid and false if one is
         * valid but the other invalid. */
        bool operator==(const InvalidBase& i) const { return invalid==i.is_invalid(); }
        /** \brief InvalidAdapter objects can be compared to InvalidBase objects.
         *
         * Returns false if both objects are valid/invalid and true if one is
         * valid but the other invalid. */
        bool operator!=(const InvalidBase& i) const { return invalid!=i.is_invalid(); }
    private:
        /** \brief Constructing InvalidAdapter objects from derived classes is
         * not allowed. */
        InvalidAdapter(const C& c) {}
        /** \brief Assigning derived classes to invalid objects is not allowed. */
        InvalidAdapter& operator=(const C& c) {}
    };

    /** \brief Convenience class to iterate over a range of integers.
     *
     * This class allows to write something like
     * @code
     for( int i : Range(11,17) ) {
         cout << "idx = " << i << endl;
     }
     * @endcode
     * to iterate from 11 to 17 inclusively or
     * @code
     for( int i : Range(100) ) {
         cout << "idx = " << i << endl;
     }
     * @endcode
     * to iterate from 0 to 99. */
    class Range {
    public:
        /** \brief Iterator class to iterate through a Range object. */
        class RangeIt {
        public:
            RangeIt(int i, int incr);
            RangeIt & operator++();
            bool operator== (const RangeIt& other) const;
            bool operator!= (const RangeIt& other) const;
            int operator*() const { return idx; }
        private:
            int idx;
            int idx_increment;
        };

        /** \brief Iterate from \e first to \e last (inclusive) with step size
         * \e increment. */
        Range(int first, int last, int increment);

        /** \brief Iterate from \e first to \e last (inclusive) with step size
         * one. */
        Range(int first, int last);

        /** \brief Iterate from zero to \e last-1 with step size one. */
        Range(int last);

        RangeIt begin() const;
        RangeIt end() const;
    private:
        int begin_idx;
        int end_idx;
        int idx_increment;
    };

    /** \brief iRange(n) counts down from n-1 to zero. */
    class iRange: public Range {
    public:
        /** \brief Iterate from \e n-1 to zero with step size one. */
        iRange(int n);
    };

    /** Enumerate allows for python-like for loops. */
    /* The code is taken from
     * http://stackoverflow.com/questions/11328264/python-like-loop-enumeration-in-c
     * and only slightly modified. */

    template<typename Iterable>
        class EnumerateObject
    {
    private:
        Iterable _iterable;
        int _counter;
        int _inc;
        decltype(_iterable.begin()) _begin;

    public:
        template<class Iterable2>
            EnumerateObject(Iterable2&& iterable, int counter, int inc):
            _iterable(std::forward<Iterable2>(iterable)),
            _counter(counter),
            _inc(inc),
            _begin(iterable.begin())
            {}

        const EnumerateObject& begin() const { return *this; }
        const EnumerateObject& end()   const { return *this; }

        bool operator!=(const EnumerateObject& other) const {
            return this->_begin != other._iterable.end();
        }

        void operator++() {
            ++_begin;
            _counter += _inc;
        }

        std::pair<int&, decltype(*_begin)> operator*() {
            return std::pair<int&, decltype(*_begin)>(_counter, *_begin);
        }
    };

    template<typename Iterable>
        EnumerateObject<Iterable> enumerate(Iterable&& iter, int counter = 0, int inc = 1) {
        return EnumerateObject<Iterable>(std::forward<Iterable>(iter), counter, inc);
    }

    //========================================================//
    //                  Global Variables                      //
    //========================================================//

    /** \brief Globale object of Invalid class.
     *
     * Use this object to assignment and testing of objects of derived
     * classes like:
     * @code
     * ...
     * @endcode */
    extern const InvalidBase INVALID;

    //========================================================//
    //      Function Definitions and Template Functions       //
    //========================================================//


    /** \brief Simple helper function that returns \a value clamped to the
     * interval [\a lower,\a upper].
     *
     * @param lower The lower bound for value.
     * @param upper The upper bound for value.
     * @param value Value that is to be clamped.
     * */
    template < typename T >
        T clamp(const T& lower, const T& upper, const T& value) {
        if(value<lower) {
            return lower;
        } else if(value>upper) {
            return upper;
        }
        return value;
    }

    /** Sum of container. */
    template < class C >
        typename C::value_type sum(const C & c) {
        typename C::value_type ret = *c.begin();
        for(auto idx_elem : enumerate(c)) {
            if(idx_elem.first==0) continue;
            ret += idx_elem.second;
        }
        return ret;
    }

    /** True if all elements are true. */
    template < class C >
        bool all(const C & c) {
        for(auto elem : c) {
            if(!elem) { return false; }
        }
        return true;
    }

    /** True if any element is true. */
    template < class C >
        bool any(const C & c) {
        for(auto elem : c) {
            if(elem) { return true; }
        }
        return false;
    }

    /** \brief Select a random element from a vector. */
    template < typename T >
        T random_select(const std::vector<T> vec) {
        if(vec.size()==0) {
            DEBUG_ERROR("Cannot draw from an empty vector");
        }
        return vec[rand()%vec.size()];
    }

    /** \brief Return index draw from normalized (or unnormalized) vector. */
    template < typename T >
        int draw_idx(const T& vec, bool normalized = true) {
        if(vec.size()==0) {
            DEBUG_ERROR("Cannot draw from an empty vector");
            return -1;
        }
        if(!normalized) {
            double sum = 0;
            for(auto& elem : vec) {
                sum += elem;
            }
            double prob = drand48();
            int idx = 0;
            for(auto& elem : vec) {
                prob -= elem/sum;
                if(prob<0) {
                    return idx;
                }
                ++idx;
            }
            DEBUG_DEAD_LINE;
        } else {
            double prob = drand48();
            int idx = 0;
            for(auto& elem : vec) {
                prob -= elem;
                if(prob<0) {
                    return idx;
                }
                ++idx;
            }
            DEBUG_DEAD_LINE;
        }
        return -1;
    }

    /** Add numbers in logarithmic scale while avoiding over flow. Computes
     * \f$ \log ( \exp t_{1} + \exp t_{2} ) \f$ by decomposing it as
     * \f$ t^{*} + \log [ \exp (t_{1}-t^{*}) + \exp (t_{2}-t^{*}) ] \f$ with
     * \f$ t^{*} = \max(t_{1},t_{2}) \f$.*/
    template < typename T >
        T log_add_exp(const T & t1, const T & t2) {
        // get max of t1 and t2
        T t_max = std::max(t1,t2);
        // argument of one exp is ==0 the other is <=0, so the sum is between 1
        // and 2
        T sum = exp(t1 - t_max) + exp(t2 - t_max);
        // log(sum) is between 0 and log(2)
        return t_max + log(sum);
    }

    /** Implements the SoftMax function. Given input vector \f$u\f$ and
     * temperature \f$T\f$ the return vector \f$v\f$ is computed as \f$v_{i} =
     * \frac{\exp\left[u_{i}/T\right]}{\sum_{j}\exp\left[u_{j}/T\right]}\f$. */
    template < typename Vec >
        Vec soft_max(const Vec& vec, double temperature) {
        if(vec.size()==0) {
            Vec ret = vec;
            return ret;
        }
        //---------------//
        // Use log scale //
        //---------------//
        double log_sum = vec[0]/temperature; // cannot initialize to log(0)
        for(int idx=1; idx<(int)vec.size(); ++idx) {
            log_sum = log_add_exp(log_sum,vec[idx]/temperature);
        }
        Vec ret = vec;
        for(int idx=0; idx<(int)vec.size(); ++idx) {
            ret[idx] = exp(vec[idx]/temperature - log_sum);
        }
        return ret;
    }
    /** SoftMax function for temperature 1. */
    template < typename Vec >
        Vec soft_max(const Vec& vec) {
        if(vec.size()==0) {
            Vec ret = vec;
            return ret;
        }
        //---------------//
        // Use log scale //
        //---------------//
        double log_sum = vec[0]; // cannot initialize to log(0)
        for(int idx=1; idx<(int)vec.size(); ++idx) {
            log_sum = log_add_exp(log_sum,vec[idx]);
        }
        Vec ret = vec;
        for(int idx=0; idx<(int)vec.size(); ++idx) {
            ret[idx] = exp(vec[idx] - log_sum);
        }
        return ret;
    }

    /** \brief Return true if t compares equal to itself. */
    template < typename T >
    inline bool is_number(T t) {
        return (t == t);
    }

    /** \brief Return true if t is finit. More specifically return true if
     * @code
     * t <= std::numeric_limits<T>::max() && t >= std::numeric_limits<T>::lowest()
     * @endcode. */
    template < typename T >
    inline bool is_finite_number(T t) {
        return (t <= std::numeric_limits<T>::max() && t >= std::numeric_limits<T>::lowest());
    }

    /** \brief Generic sign function.
     *
     * Returns -1 if val is less than T(0), +1 for greater, and 0 for equality. */
    template <class T>
        int sgn(T val) { return ( (val < T(0)) ? -1 : ( (val > T(0)) ? 1 : 0 ) ); }

    /** \brief Generic function to determine if c1 and c2 are approximately
     * equality.
     *
     * Returns true if the difference is smaller than the value returned by
     * util::approx_equal_tolerance(). */
    template < class C >
        bool approx_eq(const C& c1, const C& c2) { return fabs(c1-c2)<approx_equal_tolerance(); }

    /** \brief Generic function to determine if c1 is much greater than c2.
     *
     * Returns true if the c1 is greater than c2 by more than the value returned
     * by util::approx_equal_tolerance(). */
    template < class C >
        bool operator>>(const C& c1, const C& c2) { return c1>c2+approx_equal_tolerance(); }

    /** \brief Generic function to determine if c1 is much smaller than c2.
     *
     * Returns true if the c1 is less than c2 by more than the value returned by
     * util::approx_equal_tolerance(). */
    template < class C >
        bool operator<<(const C& c1, const C& c2) { return c1>c2-approx_equal_tolerance(); }

    //=======================================================================//
    // Define << operator for tuples (this is just copy pasted from the web) //

    namespace aux{
        template<std::size_t...> struct seq{};

        template<std::size_t N, std::size_t... Is>
            struct gen_seq : gen_seq<N-1, N-1, Is...>{};

        template<std::size_t... Is>
            struct gen_seq<0, Is...> : seq<Is...>{};

        template<class Ch, class Tr, class Tuple, std::size_t... Is>
            void print_tuple(std::basic_ostream<Ch,Tr>& os, Tuple const& t, seq<Is...>){
            using swallow = int[];
            (void)swallow{0, (void(os << (Is == 0? "" : ", ") << std::get<Is>(t)), 0)...};
        }
    } // aux::

    template<class Ch, class Tr, class... Args>
        auto operator<<(std::basic_ostream<Ch, Tr>& os, std::tuple<Args...> const& t)
        -> std::basic_ostream<Ch, Tr>&
    {
        os << "(";
        aux::print_tuple(os, t, aux::gen_seq<sizeof...(Args)>());
        return os << ")";
    }

    //=======================================================================//

} // end namespace util

#include "debug_exclude.h"

#endif /* UTIL_H_ */
