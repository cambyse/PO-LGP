/*
 * Identifier.h
 *
 *  Created on: Jun 21, 2012
 *      Author: robert
 */

#ifndef IDENTIFIER_H_
#define IDENTIFIER_H_

namespace Utils
{

template <typename T>
class Identifier1D
{
public:
    Identifier1D(const T& t): type(t) {}
    Identifier1D(const Identifier1D<T>& i): type(i.type) {}
    virtual ~Identifier1D();
    bool operator==(const Identifier1D<T>& other) const { return type==other.type; }
    bool operator< (const Identifier1D<T>& other) const { return type<other.type; }
    bool operator!=(const Identifier1D<T>& other) const { return !(*this==other); }
    bool operator<=(const Identifier1D<T>& other) const { return (*this<other || *this==other); }
    bool operator> (const Identifier1D<T>& other) const { return !(*this<=other); }
    bool operator>=(const Identifier1D<T>& other) const { return !(*this>other || *this==other); }
protected:
    T type;
};

template <typename T1, typename T2 = T1 >
class Identifier2D
{
public:
    Identifier2D(const T1& t1, const T2& t2): type1(t1), type2(t2) {}
    Identifier2D(const Identifier2D<T1,T2>& i): type1(i.type1), type2(i.type2) {}
    virtual ~Identifier2D();
    bool operator==(const Identifier2D<T1,T2>& other) const { return (type1==other.type1 && type1==other.type1); }
    bool operator< (const Identifier2D<T1,T2>& other) const { return (type1<other.type1 || ( type1==other.type1 && type2<other.type2 )); }
    bool operator!=(const Identifier2D<T1,T2>& other) const { return !(*this==other); }
    bool operator<=(const Identifier2D<T1,T2>& other) const { return (*this<other || *this==other); }
    bool operator> (const Identifier2D<T1,T2>& other) const { return !(*this<=other); }
    bool operator>=(const Identifier2D<T1,T2>& other) const { return !(*this>other || *this==other); }
protected:
    T1 type1;
    T2 type2;
};

} /* namespace Utils */
#endif /* IDENTIFIER_H_ */
