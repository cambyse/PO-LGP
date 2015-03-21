#include <FOL/relationalMachine.h>

//===========================================================================

void TEST(RM){
  RelationalMachine RM("machine.fol");

  cout <<"symbols = " <<RM.getSymbols() <<endl;
  cout <<"rules = " <<RM.getRules() <<endl;

  cout <<"state = " <<RM.getState() <<endl;
  RM.applyEffect("(init)");
  cout <<"state = " <<RM.getState() <<endl;
  RM.fwdChainRules();
  cout <<"state = " <<RM.getState() <<endl;
  RM.applyEffect("(alignHand conv) (positionHand conv)");  //msg from the actions
  cout <<"state = " <<RM.getState() <<endl;
  RM.fwdChainRules();
  cout <<"state = " <<RM.getState() <<endl;
  RM.applyEffect("(lowerHand conv)");  //msg from the actions
  cout <<"state = " <<RM.getState() <<endl;
  RM.fwdChainRules();
  cout <<"state = " <<RM.getState() <<endl;
  RM.applyEffect("(controlForce timeout)");  //msg from the actions
  cout <<"state = " <<RM.getState() <<endl;
  RM.fwdChainRules();
  cout <<"state = " <<RM.getState() <<endl;
  RM.applyEffect("(homing conv)");  //msg from the actions
  cout <<"state = " <<RM.getState() <<endl;
  RM.fwdChainRules();
  cout <<"state = " <<RM.getState() <<endl;

  RM.applyEffect("(undefined)");  //generates error

}

//===========================================================================

int main(int argn, char** argv){
  testRM();
}
