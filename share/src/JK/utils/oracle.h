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

class HumanOracle : public Oracle {
	private:
		const char* predicate;
  public:
		HumanOracle(const char* predicate);
    virtual const int classify(const MT::Array<arr>& data, const int set = 0) const;
};

class Oracle_GroundedSymbol : public relational::GroundedSymbol {
	public:
		Oracle_GroundedSymbol(MT::String name, uint arity, bool build_derived_predicates = false) : GroundedSymbol(name, arity, build_derived_predicates) {};
		virtual bool holds(arr& x) const;
};
#endif //_ORACLE_H_
