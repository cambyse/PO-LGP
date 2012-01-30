#ifndef _ORACLE_H_
#define _ORACLE_H_

#include <MT/array.h>

class Oracle {
  public:
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const = 0;
    
};

class OnOracle : public Oracle {
  public:
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const;
};

class HumanOracle : public Oracle {
  public:
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const;
};
#endif //_ORACLE_H_
