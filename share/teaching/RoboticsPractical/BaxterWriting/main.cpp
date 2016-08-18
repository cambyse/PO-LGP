#include "MyBaxter.h"

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  {
    MyBaxter baxter;
    arr y = ARR(0.6, -0.1, 1.1);

    //Align endeffector with base
    CtrlTask *alignXZ = baxter.align("alignXZ", "endeffL", Vector_x, "base_footprint", Vector_z,-1);
    CtrlTask *alignYY = baxter.align("alignYY", "endeffL", Vector_y, "base_footprint", Vector_y, 1);
    CtrlTask *alignZX = baxter.align("alignZX", "endeffL", Vector_z, "base_footprint", Vector_x, 1);

    //get right arm out of the way
    CtrlTask *pos_eeR =  baxter.goToPosition("endeffR","base_footprint", ARR(0.8, -0.6, 1.4), 1);
    mlr::wait(1);

    //move right arm above the starting position
    CtrlTask *pos_eeL =  baxter.goToPosition("endeffL","base_footprint", y, 1);
    mlr::wait(2.);

    baxter.findTheTable(pos_eeL, y);
    mlr::wait(2);

    baxter.stop({alignXZ, alignYY, alignZX });

    ofstream myfile;
    myfile.open("writeBaxter.txt");

    arr qFirst;
    baxter.getState(qFirst, NoArr, NoArr);
    myfile<< qFirst <<" 0"<<endl;
    baxter.setTestJointState(qFirst);

    // switch to torque control
    baxter.disablePosControlL();
    mlr::wait(1.);    

    baxter.useRos(false); //while simulating

//----- Write B:
    arr nrStepsB = ARR(30, 20, 10, 20, 20, 10, 20);

    arr delta_yB = { -0.1,  0.0 ,  0.0,
                     0.01,  0.05,  0.0,
                     0.03,  0.0,   0.0,
                     0.01, -0.05,   0.0,
                     0.01,  0.05,  0.0,
                     0.03,  0.0,  0.0,
                     0.01, -0.05,  0.0};
    delta_yB.reshape(7,3);
    arr forceB = ARR( 1, 1, 1, 1, 1, 1, 1);

    baxter.writeLetterSim(myfile, delta_yB, nrStepsB, forceB, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yB, nrStepsB, forceB, *pos_eeL);
// ------

// ----- Move to A start:
    arr nrStepsAm = ARR(10, 30, 10);

    arr delta_yAm = {  0.0,   0.0 ,  0.04,
                       0.0,   0.07,  0.0,
                       0.0,   0.0,  -0.04};
    delta_yAm.reshape(3,3);
    arr forceAm = ARR( 0, 0, 0);

    baxter.writeLetterSim(myfile, delta_yAm, nrStepsAm, forceAm, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yAm, nrStepsAm, forceAm, *pos_eeL);
// ------

//----- Write A:
     arr nrStepsA = ARR(30, 30, 10, 15, 10, 20);

     arr delta_yA = {-0.1,   0.0 ,  0.0,
                      0.1,   0.06,  0.0,
                      0.0,   0.0,   0.04,
                     -0.02, -0.06,  0.0,//-0.05, -0.06,  0.0,
                      0.0,   0.0,  -0.04,
                      0.0,   0.05,  0.0};
    delta_yA.reshape(6,3);
    arr forceA = ARR( 1, 1, 0, 0, 0, 1);

    baxter.writeLetterSim(myfile, delta_yA, nrStepsA, forceA, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yA, nrStepsA, forceA, *pos_eeL);
// ------

//----- Move to X start
    arr nrStepsXm = ARR(10, 20, 10);

    arr delta_yXm = {  0.0,   0.0 ,  0.04,
                      0.05,  0.01,  0.0,
                      0.0,   0.0,  -0.04};
    delta_yXm.reshape(3,3);
    arr forceXm = ARR( 0, 0, 0);

    baxter.writeLetterSim(myfile, delta_yXm, nrStepsXm, forceXm, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yXm, nrStepsXm, forceXm, *pos_eeL);
// ------

// ----- Write X:
    arr nrStepsX = ARR(30, 10, 15, 10, 30);

    arr delta_yX = {-0.1,   0.06 , 0.0,
                     0.0,   0.0,   0.04,
                     0.1,   0.0,   0.0,
                     0.0,   0.0,  -0.04,
                    -0.1,  -0.06,  0.0};
    delta_yX.reshape(5,3);
    arr forceX = ARR( 1, 0, 0, 0, 1);

    baxter.writeLetterSim(myfile, delta_yX, nrStepsX, forceX, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yX, nrStepsX, forceX, *pos_eeL);
// ------

// Move to T start:
   arr nrStepsTm = ARR(10, 30, 10);

   arr delta_yTm = {  0.0,   0.0 ,  0.04,
                      0.0,   0.05,   0.0,
                      0.0,   0.0,  -0.04};
   delta_yTm.reshape(3,3);
   arr forceTm = ARR( 0, 0, 0);

    baxter.writeLetterSim(myfile, delta_yTm, nrStepsTm, forceTm, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yTm, nrStepsTm, forceTm, *pos_eeL);
// ------

//--------- Write T:
    arr nrStepsT = ARR(30, 10, 15, 10, 30);

    arr delta_yT = { 0.0,   0.09 ,  0.0,
                     0.0,   0.0,   0.04,
                     0.0,  -0.04, 0.0,
                     0.0,   0.0,  -0.04,
                     0.1,   0.0,   0.0};
    delta_yT.reshape(5,3);
    arr forceT = ARR( 1, 0, 0, 0, 1);

    baxter.writeLetterSim(myfile, delta_yT, nrStepsT, forceT, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yT, nrStepsT, forceT, *pos_eeL);
// ------

// Move to E start:
    arr nrStepsEm = ARR(10, 30, 10);

    arr delta_yEm = {  0.0,   0.0 ,  0.04,
                      0.0,   0.09, 0.0,
                      0.0,   0.0,  -0.04};
    delta_yEm.reshape(3,3);
    arr forceEm = ARR( 0, 0, 0);

    baxter.writeLetterSim(myfile, delta_yEm, nrStepsEm, forceEm, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yEm, nrStepsEm, forceEm, *pos_eeL);
// ------

//--------- Write E:
    arr nrStepsE = ARR(20, 30, 20, 10, 15, 10, 20);

    arr delta_yE = { 0.0,  -0.05,  0.0,
                    -0.1,   0.0,   0.0,
                     0.0,   0.05,  0.0,
                     0.0,   0.0,   0.04,
                     0.05, -0.05,  0.0,
                     0.0,   0.0,  -0.04,
                     0.0,   0.05,  0.0};
    delta_yE.reshape(7,3);
    arr forceE = ARR( 1, 1, 1, 0, 0, 0, 1);

    baxter.writeLetterSim(myfile, delta_yE, nrStepsE, forceE, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yE, nrStepsE, forceE, *pos_eeL);
// ------

// Move to R start:
   arr nrStepsRm = ARR(10, 20, 10);

   arr delta_yRm = {  0.0,   0.0 ,  0.04,
                      0.05,  0.04,  0.0,
                      0.0,   0.0,  -0.04};
   delta_yRm.reshape(3,3);
   arr forceRm = ARR( 0, 0, 0);


    baxter.writeLetterSim(myfile, delta_yRm, nrStepsRm, forceRm, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yRm, nrStepsRm, forceRm, *pos_eeL);
// ------

//--------- Write R:
    arr nrStepsR = ARR(30, 20, 15, 20, 20, 10);

    arr delta_yR = {-0.1,   0.0 ,  0.0,
                    0.01,   0.05,  0.0,
                    0.03,   0.0,   0.0,
                    0.01,  -0.05,  0.0,
                    0.05,   0.05,  0.0,
                    0.0,    0.0,   0.04};
   delta_yR.reshape(6,3);
   arr forceR = ARR( 1, 1, 1, 1, 1, 0);

    baxter.writeLetterSim(myfile, delta_yR, nrStepsR, forceR, *pos_eeL, qFirst);
    //baxter.writeLetterReal(delta_yR, nrStepsR, forceR, *pos_eeL);
// ------

    myfile.close();

    baxter.useRos(true);

    arr Data = FILE("writeBaxter.txt");
    baxter.writeLettersData(Data, *pos_eeL);

    baxter.enablePosControlL();

    baxter.homing();
  }

  cout <<"bye bye" <<endl;
  return 0;
}

