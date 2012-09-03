#ifndef _ORACLE_H_
#define _ORACLE_H_

#include <relational/symbolGrounding.h>

#include <MT/array.h>

class Oracle {
  public:
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const = 0;
    
};

class OnOracle : public Oracle {
  public:
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const;
};

class CloseOracle : public Oracle {
  public:
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const;
};

class HigherOracle : public Oracle {
  public:
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const;
};

class InsideOracle : public Oracle {
  public:
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const;
};

class HumanOracle : public Oracle {
	private:
		const char* predicate;
  public:
		HumanOracle(const char* predicate);
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const;
};

#endif //_ORACLE_H_
