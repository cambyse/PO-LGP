struct Variable;
struct Rule;
typedef MT::Array<Variable*> VariableList;
typedef MT::Array<Rule*> RuleList;

VariableList globalVars;

struct Variable {
  uint id;          //!< unique identifyer
  uint dim;         //!< cardinality of domain
  MT::String name;  //!< up to you...
  MT::Array<MT::String> valueNames;
  
  Variable();
  Variable(uint _dim, const char *_name);
  Variable(uint _dim, const char *_name, uint _id);
  ~Variable();
  operator uint() const { return id; }
  void write(std::ostream& os = std::cout) const;
  void read(std::istream& is);
};

struct Rule {
  VariableList vars;
  boolA negation;
  uintA values;
  uint arrow; //preconditionSize
  double weight;
  
  void write(std::ostream& os = std::cout) const;
  void read(std::istream& is);
};

#if 1
#  include "rules.cpp"
#endif
