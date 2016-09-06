#include "gravityCompensation.h"
#include <Algo/MLcourse.h>
#include <Motion/taskMaps.h>

struct GravityCompensation::CV : public CrossValidation {
  void  train(const arr& X, const arr& y, double param, arr& beta) {
    beta = ridgeRegression(X, y, param); //returns optimal beta for training data
  }
  double test(const arr& X, const arr& y, const arr& beta) {
    arr y_pred = X*beta;
    return sqrt(sumOfSqr(y_pred-y)/y.N); //returns RMSE on test data
  }

  arr calculateBetaWithCV(const arr& Phi, const arr& y, const arr& lambdas, const bool& verbose, double& c) {
    this->verbose = verbose;
    this->crossValidateMultipleLambdas(Phi, y, lambdas, 10, false);

    if(verbose) {
      this->plot();
    }
    if(verbose) cout <<"10-fold CV:\n  costMeans= " << this->scoreMeans << "\n  costSDVs= " << this->scoreSDVs << endl;

    uint bestIndex = this->scoreMeans.minIndex();

    c = (1-scoreMeans(bestIndex)/scoreMeans.last())*100;

    return ridgeRegression(Phi, y, lambdas(bestIndex));
  }
};

void GravityCompensation::learnGCModel() {
  GravityCompensation::CV cv;

  arr q;
  q << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_LW/q"));
  arr u;
  u << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_LW/u"));
  arr qSign;
  qSign << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_LW/qSign"));

  arr lambdas = ARR(1e-3,1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5);
  lambdas.append(lambdas*5.0);
  lambdas.append(lambdas*7.0);
  lambdas.append(lambdas*3.0);
  lambdas.sort(); //not necessary, just for nicer cv plots


  double c;
  mlr::String joint = "l_shoulder_pan_joint";
  uint index = world.getJointByName(joint)->qIndex;
  beta_l_shoulder_pan_joint = cv.calculateBetaWithCV(featuresGC(q, qSign, joint), u.sub(0,-1,index,index), lambdas, false, c);
  cout << c << endl;

  joint = "l_shoulder_lift_joint";
  index = world.getJointByName(joint)->qIndex;
  beta_l_shoulder_lift_joint = cv.calculateBetaWithCV(featuresGC(q, qSign, joint), u.sub(0,-1,index,index), lambdas, false, c);
  cout << c << endl;

  joint = "l_upper_arm_roll_joint";
  index = world.getJointByName(joint)->qIndex;
  beta_l_upper_arm_roll_joint = cv.calculateBetaWithCV(featuresGC(q, qSign, joint), u.sub(0,-1,index,index), lambdas, false, c);
  cout << c << endl;

  joint = "l_elbow_flex_joint";
  index = world.getJointByName(joint)->qIndex;
  beta_l_elbow_flex_joint = cv.calculateBetaWithCV(featuresGC(q, qSign, joint), u.sub(0,-1,index,index), lambdas, false, c);
  cout << c << endl;

  joint = "l_forearm_roll_joint";
  index = world.getJointByName(joint)->qIndex;
  beta_l_forearm_roll_joint = cv.calculateBetaWithCV(featuresGC(q, qSign, joint), u.sub(0,-1,index,index), lambdas, false, c);
  cout << c << endl;

  joint = "l_wrist_flex_joint";
  index = world.getJointByName(joint)->qIndex;
  beta_l_wrist_flex_joint = cv.calculateBetaWithCV(featuresGC(q, qSign, joint), u.sub(0,-1,index,index), lambdas, false, c);
  cout << c << endl;

}

arr GravityCompensation::compensate(const arr& q, const arr& qSign, const StringA& joints) {
  arr u = zeros(world.getJointStateDimension());

  for(mlr::String joint : joints) {
    uint index = world.getJointByName(joint)->qIndex;
    arr beta = zeros(1);
    if(joint == "l_shoulder_pan_joint") {
      beta = beta_l_shoulder_pan_joint;
    } else if(joint == "l_shoulder_lift_joint") {
      beta = beta_l_shoulder_lift_joint;
    } else if(joint == "l_upper_arm_roll_joint") {
      beta = beta_l_upper_arm_roll_joint;
    } else if(joint == "l_elbow_flex_joint") {
      beta = beta_l_elbow_flex_joint;
    } else if(joint == "l_forearm_roll_joint") {
      beta = beta_l_forearm_roll_joint;
    } else if(joint == "l_wrist_flex_joint") {
      beta = beta_l_wrist_flex_joint;
    }
    arr uTemp = featuresGC(q, qSign, joint)*beta;
    u(index) = uTemp(0,0);
  }
  clip(u, -7.0, 7.0); //TODO: More sophisticated clipping!
  return u;
}

void GravityCompensation::learnFTModel() {
  GravityCompensation::CV cv;

  arr q;
  q << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_05_09_16/q"));
  arr fL;
  fL << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_05_09_16/fL"));
  arr fR;
  fR << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_05_09_16/fR"));

  arr PhiL = featuresFT(q, "endeffL");
  arr PhiR = featuresFT(q, "endeffR");

  arr lambdas = ARR(1e-3,1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5);
  lambdas.append(lambdas*5.0);
  lambdas.append(lambdas*7.0);
  lambdas.append(lambdas*3.0);
  lambdas.sort(); //not necessary, just for nicer cv plots

  arr mL = zeros(6);
  arr mR = zeros(6);
  for(uint i = 0; i < 6; i++) {
    betaFTL.append(~cv.calculateBetaWithCV(PhiL, fL.sub(0,-1,i,i), lambdas, false, mL(i)));
    betaFTR.append(~cv.calculateBetaWithCV(PhiR, fR.sub(0,-1,i,i), lambdas, false, mR(i)));
  }
  betaFTL = ~betaFTL;
  betaFTR = ~betaFTR;
  cout << "Percentage of variance for left endeff FT sensor:" << mL << endl;
  cout << "Percentage of variance for right endeff FT sensor: " <<mR << endl;
}

arr GravityCompensation::compensateFTL(const arr& q) {
  return featuresFT(q, "endeffL")*betaFTL;
}

arr GravityCompensation::compensateFTR(const arr& q) {
  return featuresFT(q, "endeffR")*betaFTR;
}

GravityCompensation::GravityCompensation(const ors::KinematicWorld& world) : world(world) {
  TLeftArm = zeros(leftJoints.N, world.getJointStateDimension());
  for(uint i = 0; i < leftJoints.N; i++) {
    TLeftArm(i, world.getJointByName(leftJoints(i))->qIndex) = 1;
  }
  TRightArm = zeros(rightJoints.N, world.getJointStateDimension());
  for(uint i = 0; i < rightJoints.N; i++) {
    TRightArm(i, world.getJointByName(rightJoints(i))->qIndex) = 1;
  }
  THead = zeros(headJoints.N, world.getJointStateDimension());
  for(uint i = 0; i < headJoints.N; i++) {
    THead(i, world.getJointByName(headJoints(i))->qIndex) = 1;
  }
}

arr GravityCompensation::featuresGC(arr q, arr qSign, const mlr::String& joint) {
  arr Phi;

  if(q.nd < 2) {
    q = ~q;
    qSign = ~qSign;
  }

  uint index = world.getJointByName(joint)->qIndex;

  arr X = q*~TLeftArm;

  Phi = makeFeatures(X, linearFT);

  arr Phi_tmp;
  arr phi_t;
  for (uint t = 0; t < q.d0; t++) {
    phi_t.clear();
    world.setJointState(q[t], q[t]*0.);

    arr M,F;
    world.equationOfMotion(M,F);
    phi_t.append(TLeftArm*F);

    Phi_tmp.append(~phi_t);
  }

  Phi = catCol(Phi,Phi_tmp);

  //Phi = makeFeatures(X,quadraticFT);

  // add sin/cos features
  Phi = catCol(Phi,sin(X));
  Phi = catCol(Phi,cos(X));

  Phi = catCol(Phi, sign(qSign.sub(0,-1,index,index)));

  return Phi;

}

arr GravityCompensation::featuresFT(arr q, mlr::String endeff) {
  if(q.nd < 2) q = ~q;

  arr Phi = ones(q.d0,1);
  Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(vecTMT, world, endeff, ors::Vector(1.0,0.0,0.0)), q));
  Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(vecTMT, world, endeff, ors::Vector(0.0,1.0,0.0)), q));
  Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(vecTMT, world, endeff, ors::Vector(0.0,0.0,1.0)), q));
  return Phi;
}

arr GravityCompensation::generateTaskMapFeature(TaskMap_Default map, arr Q) {
  arr phiTemp;
  for (uint t = 0; t < Q.d0; t++) {
    world.setJointState(Q[t]);
    arr y;
    map.phi(y, NoArr, world);
    phiTemp.append(~y);
  }
  return phiTemp;
}







void GravityCompensation::testForLimits() {
  arr q;
  q << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_05_09_16/q"));
  //bool limitViolation = false;
  uint j = 0;
  for(uint i = 0; i < q.d0; i++) {
    world.setJointState(q[i]);
    LimitsConstraint limits(0.03);
    arr y;
    limits.phi(y,NoArr,world);
    if(y(0) > 0) {
      cout << i << " " << y<< endl;
      j++;
    }
  }
  cout << j << endl;

}

void GravityCompensation::removeLimits() {
  arr q;
  q << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_05_09_16/q"));
  arr u;
  u << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_05_09_16/u"));
  arr qSign;
  qSign << FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_05_09_16/qSign"));

  arr qNew, uNew, qSignNew;

  for(uint i = 0; i < q.d0; i++) {
    world.setJointState(q[i]);
    LimitsConstraint limits;
    arr y;
    limits.phi(y, NoArr, world);
    if(y(0) <= 0) {
      qNew.append(~q[i]);
      qSignNew.append(~qSign[i]);
      uNew.append(~u[i]);
    }
  }
  FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_LW/q")) << qNew;
  FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_LW/qSign")) << qSignNew;
  FILE(mlr::mlrPath("examples/pr2/calibrateControl/logData/gcKugel_LW/u")) << uNew;
}









#if 0

arr GravityCompensation::generateTaskMapFeature(TaskMap_Default map, arr Q) {
  arr phiTemp;
  for (uint t = 0; t < Q.d0; t++) {
    world.setJointState(Q[t]);
    arr y;
    map.phi(y, NoArr, world);
    phiTemp.append(~y);
  }
  return phiTemp;
}

void GravityCompensation::generatePredictionsOnDataSet(const arr& Q, const arr& U, const StringA& joints) {
  if(!modelLearned) {
    learnModels(false);
  }
  for(mlr::String s : joints) {
    uint index = world.getJointByName(s)->qIndex;
    arr UPred;
    for(uint i = 0; i < Q.d0; i++) {
      world.setJointState(Q[i]);
      UPred.append(compensate(world.getJointState(), true, true, true)(index));
    }
    FILE("joints.dat") << joints;
    FILE(STRING(s << "_uPred.dat")) << UPred;
    FILE(STRING(s << "_u.dat")) << U.sub(0,-1,index,index);
  }
}

arr GravityCompensation::features(arr Q, const GravityCompensation::RobotPart robotPart) {
  arr Phi;

  if(Q.nd < 2) {
    Q = ~Q;
  }

  CHECK(Q.d1 == world.getJointStateDimension(), "Wrong Q dimension")

      if(robotPart == leftArm) {
    arr X = Q*~TLeftArm;

    Phi = makeFeatures(X, linearFT);

    arr Phi_tmp;
    arr phi_t;
    for (uint t = 0; t < Q.d0; t++) {
      phi_t.clear();
      world.setJointState(Q[t], Q[t]*0.);

      arr M,F;
      world.equationOfMotion(M,F);
      phi_t.append(TLeftArm*F);

      Phi_tmp.append(~phi_t);
    }

    Phi = catCol(Phi,Phi_tmp);



    //Phi = makeFeatures(X,quadraticFT);

    // add sin/cos features
    Phi = catCol(Phi,sin(X));
    Phi = catCol(Phi,cos(X));

    /*
    //Summed sinus/cosinus
    arr temp;
    for(uint i = 0; i < leftJoints.N; i++) {
      temp = X.sub(0,-1,i,-1);
      Phi = catCol(Phi,sin(sum(temp,1)));
      Phi = catCol(Phi,cos(sum(temp,1)));
    }
*/
    /*
    TaskMap_Default posTask(posTMT, world, "endeffR");
      Phi_tmp.clear();
      for (uint t = 0; t<Q.d0; t++) {
        world.setJointState(Q[t],Q[t]*0.);
        arr yPos;
        posTask.phi(yPos, NoArr, world);
        Phi_tmp.append(~yPos);
      }
      Phi = catCol(Phi,Phi_tmp);*/



    //Different Body Parts TODO: add more, if one like
    //Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(posTMT, world, "endeffL"), Q));
    //Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(vecTMT, world,"endeffL",ors::Vector(1.,0.,0.)), Q));

    //Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(posTMT, world, "l_forearm_link_0"), Q));
    //Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(vecTMT, world,"l_forearm_link_0",ors::Vector(1.,0.,0.)), Q));



    /*
    // add dynamics features
    arr Phi_tmp;
    arr phi_t;
    for (uint t = 0; t < Q.d0; t++) {
      phi_t.clear();
      world.setJointState(Q[t], Q[t]*0.);

      arr M,F;
      world.equationOfMotion(M,F);
      phi_t.append(TLeftArm*F);

      Phi_tmp.append(~phi_t);
    }

    Phi = catCol(Phi,Phi_tmp);*/

    return Phi;

  } else if(robotPart == rightArm) {
    arr X = Q*~TRightArm;

    Phi = makeFeatures(X, linearFT);

    // add sin/cos features
    Phi = catCol(Phi,sin(X));
    Phi = catCol(Phi,cos(X));

    /*

    //Summed sinus/cosinus
    arr temp;
    for(uint i = 0; i < rightJoints.N; i++) {
      temp = X.sub(0,-1,i,-1);
      Phi = catCol(Phi,sin(sum(temp,1)));
      Phi = catCol(Phi,cos(sum(temp,1)));
    }
*/


    // add dynamics features
    arr Phi_tmp;
    arr phi_t;
    for (uint t = 0; t < Q.d0; t++) {
      phi_t.clear();
      world.setJointState(Q[t], Q[t]*0.);

      arr M,F;
      world.equationOfMotion(M,F);
      phi_t.append(TRightArm*F);

      Phi_tmp.append(~phi_t);
    }

    Phi = catCol(Phi,Phi_tmp);



    //Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(posTMT, world, "endeffR"), Q));
    //Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(vecTMT, world,"endeffR",ors::Vector(1.,0.,0.)), Q));

    //Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(posTMT, world, "r_forearm_link_0"), Q));
    //Phi = catCol(Phi, generateTaskMapFeature(TaskMap_Default(vecTMT, world,"r_forearm_link_0",ors::Vector(1.,0.,0.)), Q));

    return Phi;

  } else if(robotPart == head) {
    arr X;

    X = Q*~THead;

    Phi = makeFeatures(X, linearFT);
    //Phi = catCol(Phi,sin(X));
    //Phi = catCol(Phi,cos(X));

    /* Phi = makeFeatures(X,quadraticFT);

    // add sin/cos features
    Phi = catCol(Phi,sin(X));
    Phi = catCol(Phi,cos(X));

    //TODO dynamics feature has no effect because gravity compensations seems not to work for the head
    // add dynamics features
    /* arr Phi_tmp;
    for (uint t = 0; t < Q.d0; t++) {
      world.setJointState(Q[t], Q[t]*0.);

      arr M,F;
      world.equationOfMotion(M,F);
      Phi_tmp.append(~(THead*F));
    }
    Phi = catCol(Phi,Phi_tmp);
    */

    return Phi;
  }
  return ARR(0.0);
}

struct CV : public CrossValidation {
  void  train(const arr& X, const arr& y, double param, arr& beta) {
    beta = ridgeRegression(X, y, param); //returns optimal beta for training data
  }
  double test(const arr& X, const arr& y, const arr& beta) {
    arr y_pred = X*beta;
    return sqrt(sumOfSqr(y_pred-y)/y.N); //returns RMSE on test data
  }

  void calculateBetaWithCV(arr& optimalBeta, const StringA& joints, const arr& Phi, const arr& lambdas, const arr& Q, const arr& U, ors::KinematicWorld& world, bool verbose, arr& m) {
    m = zeros(joints.N);
    //double su = 0.0;

    for (uint i = 0; i < joints.N; i++) {
      arr y;
      y = U.col(world.getJointByName(joints(i))->qIndex);

      //cross valide
      this->crossValidateMultipleLambdas(Phi, y, lambdas, 10, false);

      if(verbose) {
        this->plot();
      }

      if(verbose) cout <<"10-fold CV:\n  costMeans= " << this->scoreMeans << "\n  costSDVs= " << this->scoreSDVs << endl;

      uint bestIndex = this->scoreMeans.minIndex();

      if(verbose) cout << "best lambda for " << joints(i) << " = " << lambdas(bestIndex) << endl;

      double c = (1-scoreMeans(bestIndex)/scoreMeans.last())*100;
      //cout << joints(i) << ": \t";
      //cout << c << endl;

      m(i) = c;

      //su += c;

      arr beta = ridgeRegression(Phi, y, lambdas(bestIndex));
      if(optimalBeta.N == 0) {
        optimalBeta = beta;
      } else {
        optimalBeta = catCol(optimalBeta, beta);
      }

      if(verbose) world.watch(true, joints(i));
    }
    //cout << endl<< endl<< su << endl;
  }
};

void GravityCompensation::learnModels(bool verbose) {
  //TODO: very ugly, make parameter file!!!!!!!!
  arr Q;
  Q << FILE(mlr::mlrPath("examples/pr2/calibrateControl/Q.dat"));
  arr U;
  U << FILE(mlr::mlrPath("examples/pr2/calibrateControl/U.dat"));

  arr PhiLeft = features(Q, leftArm);
  arr PhiRight = features(Q, rightArm);
  arr PhiHead = features(Q, head);

  CV cv;

  //cv.verbose = false;

  arr lambdas = ARR(1e-3,1e-2,1e-1,1e0,1e1,1e2,1e3,1e4,1e5);
  lambdas.append(lambdas*5.0);
  lambdas.append(lambdas*7.0);
  lambdas.append(lambdas*3.0);
  lambdas.sort(); //not necessary, just for nicer cv plots

  //FILE("leftJoints.dat") << leftJoints;

  arr m;

  cv.calculateBetaWithCV(betaLeftArm, leftJoints, PhiLeft, lambdas, Q, U, world, verbose, m);
  //FILE("leftArmErrors.dat") << m;
  cout << m << endl;

  cv.calculateBetaWithCV(betaRightArm, rightJoints, PhiRight, lambdas, Q, U, world, verbose, m);
  cout << m << endl;
  cv.calculateBetaWithCV(betaHead, headJoints, PhiHead, lambdas, Q, U, world, verbose, m);
  cout << m << endl;
  cout << "modelLearned" << endl;
  modelLearned = true;

}

void GravityCompensation::saveBetas() {
  CHECK(modelLearned, "You have to learn the model first!")

      write(LIST<arr>(betaLeftArm),STRING("betaLeftArm.dat"));
  write(LIST<arr>(betaRightArm),STRING("betaRightArm.dat"));
  write(LIST<arr>(betaHead),STRING("betaHead.dat"));
}

void GravityCompensation::loadBetas() {
  betaLeftArm << FILE(mlr::mlrPath("examples/pr2/calibrateControl/betaLeftArm.dat"));
  betaRightArm << FILE(mlr::mlrPath("examples/pr2/calibrateControl/betaRightArm.dat"));
  betaHead << FILE(mlr::mlrPath("examples/pr2/calibrateControl/betaHead.dat"));
  modelLearned = true;
}

arr GravityCompensation::compensate(arr q, bool compensateLeftArm, bool compensateRightArm, bool compensateHead) {

  CHECK(modelLearned, "You have to learn the model first!")
      CHECK(q.N == world.getJointStateDimension(), "wrong joint state dimension!")

      arr u = zeros(world.getJointStateDimension());
  if(compensateLeftArm) {
    u += features(q, leftArm)*betaLeftArm*TLeftArm;
  }
  if(compensateRightArm) {
    u += features(q, rightArm)*betaRightArm*TRightArm;
  }
  if(compensateHead) {
    u += features(q, head)*betaHead*THead;
  }

  clip(u, -7.0, 7.0); //TODO: More sophisticated clipping!

  return u;
}

arr GravityCompensation::compensate(arr q, StringA joints) {
  CHECK(modelLearned, "You have to learn the model first!")
      CHECK(q.N == world.getJointStateDimension(), "wrong joint state dimension!")

      arr uTemp = zeros(world.getJointStateDimension());
  uTemp += features(q, leftArm)*betaLeftArm*TLeftArm;
  uTemp += features(q, rightArm)*betaRightArm*TRightArm;
  uTemp += features(q, head)*betaHead*THead;

  arr u = zeros(world.getJointStateDimension());
  for(mlr::String joint : joints) {
    uint index = world.getJointByName(joint)->qIndex;
    u(index) = uTemp(index);
  }
  clip(u, -7.0, 7.0); //TODO: More sophisticated clipping!
  return u;
}

GravityCompensation::GravityCompensation(const ors::KinematicWorld& world) : world(world) {
  TLeftArm = zeros(leftJoints.N, world.getJointStateDimension());
  for(uint i = 0; i < leftJoints.N; i++) {
    TLeftArm(i, world.getJointByName(leftJoints(i))->qIndex) = 1;
  }
  TRightArm = zeros(rightJoints.N, world.getJointStateDimension());
  for(uint i = 0; i < rightJoints.N; i++) {
    TRightArm(i, world.getJointByName(rightJoints(i))->qIndex) = 1;
  }
  THead = zeros(headJoints.N, world.getJointStateDimension());
  for(uint i = 0; i < headJoints.N; i++) {
    THead(i, world.getJointByName(headJoints(i))->qIndex) = 1;
  }
}

void GravityCompensation::testForLimits() {
  arr Q;
  Q << FILE(mlr::mlrPath("examples/pr2/calibrateControl/Q.dat"));
  bool limitViolation = false;
  for(uint i = 0; i < Q.d0; i++) {
    world.setJointState(Q[i]);
    TaskMap_qLimits limits;
    arr y;
    limits.phi(y,NoArr,world);
    if(y(0) >= 1.0) {
      limitViolation = true;
    }
    cout << i << " " << y<< endl;
  }
  if(limitViolation) {
    cout << "limits violated" << endl;
  }
}

arr GravityCompensation::makeQMatrix(arr Q, uint jointIndex) {
  arr QJointStateMatrix = zeros(Q.d0,world.getJointStateDimension());

  for(uint i = 0; i < Q.d0; i++) {
    QJointStateMatrix[i] = world.getJointState();
    QJointStateMatrix[i](jointIndex) = Q(i,0);
  }

  return QJointStateMatrix;
}

#endif



