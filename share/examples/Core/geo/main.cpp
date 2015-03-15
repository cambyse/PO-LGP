#include <Core/geo.h>

//===========================================================================
//
// test very basics
//

#define TEST_DIFF_ZERO(expr) { CHECK_ZERO((expr).diffZero(), 1e-6, "TEST FAILED"); cout <<"Success: " <<(expr).diffZero() <<" == " <<#expr <<endl; }

#define TEST_ZERO(expr) { CHECK_ZERO((expr), 1e-6, "TEST FAILED"); cout <<"Success: " <<(expr) <<" == " <<#expr <<endl; }

void TEST(Basics){
  for(uint k=0;k<10;k++){
    ors::Quaternion A,B,C;
    A.setRandom();
    B.setRandom();
    C.setRandom();
    TEST_DIFF_ZERO(Quaternion_Id);
    TEST_DIFF_ZERO(A/A);
    TEST_DIFF_ZERO(A*B/B/A);
    TEST_ZERO( ((A/B).getRad()) - ((B/A).getRad()) );
  }

  for(uint k=0;k<10;k++){
    ors::Transformation A,B,C;
    A.setRandom();
    B.setRandom();
    C.setDifference(A,B);
    TEST_DIFF_ZERO(A/A);
    TEST_DIFF_ZERO(A*A/A/A);
    TEST_DIFF_ZERO(A*C/B);
  }
}

//===========================================================================

int MAIN(int argc,char **argv){

  testBasics();

  return 0;
}
