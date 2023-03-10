struct StateVariable;
struct Rule;
typedef mlr::Array<StateVariable*> StateVariableList;
typedef mlr::Array<Rule*> RuleList;

StateVariableList globalVars;

struct StateVariable {
  uint id;          ///< unique identifyer
  uint dim;         ///< cardinality of domain
  mlr::String name;  ///< up to you...
  mlr::Array<mlr::String> valueNames;
  
  StateVariable();
  StateVariable(uint _dim, const char *_name);
  StateVariable(uint _dim, const char *_name, uint _id);
  ~StateVariable();
  operator uint() const { return id; }
  void write(std::ostream& os = std::cout) const;
  void read(std::istream& is);
};
stdPipes(StateVariable);

struct Rule {
  StateVariableList vars;
  boolA negation;
  uintA values;
  uint arrow; //preconditionSize
  double weight;
  
  void write(std::ostream& os = std::cout) const;
  void read(std::istream& is);
};
stdPipes(Rule);

#if 1
#  include "rules.cpp"
#endif
