#include <FOL/relationalMachine.h>

//===========================================================================

void TEST(RM){
  RelationalMachine RM("machine.fol");

  cout <<"symbols = " <<RM.getSymbols() <<endl;
  cout <<"rules = " <<RM.getRules() <<endl;
  cout <<"state = " <<RM.getState() <<endl;

  RM <<"(init)";
  RM.fwdChainRules();

  RM <<"(alignHand conv) (positionHand conv)";  //return msg from the actions
  RM.fwdChainRules();

  RM <<"(lowerHand conv)";  //return msg from the actions
  RM.fwdChainRules();

  cout <<"(homing) condition?" <<RM.queryCondition("(homing)") <<endl;

  RM <<"(controlForce timeout)";  //return msg from the actions
  RM.fwdChainRules();

  cout <<"(homing) condition?" <<RM.queryCondition("(homing)") <<endl;

  RM <<"(homing conv)";  //return msg from the actions
  RM.fwdChainRules();

  RM <<"(undefined)";  //generates error

}

//===========================================================================

int main(int argn, char** argv){
  testRM();
}
