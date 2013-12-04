#include "pfc.h"

Pfc::Pfc(ors::KinematicWorld &_orsG, arr& _trajRef, double _TRef, arr &_x0, MObject &_goalMO, \
        bool _useOrientation, bool _useCollAvoid, \
        double _fPos_deviation, double _fVec_deviation, double _yCol_deviation, double _w_reg):
        orsG(&_orsG),
        TRef(_TRef),
        x0(_x0),
        goalMO(&_goalMO),
        useOrientation(_useOrientation),
        useCollAvoid(_useCollAvoid),
        fPos_deviation(_fPos_deviation),
        fVec_deviation(_fVec_deviation),
        yCol_deviation(_yCol_deviation),
        w_reg(_w_reg)
{
  goalRef = _trajRef[_trajRef.d0-1];
  dt = TRef/(_trajRef.d0-1);
  dsRef = 1./(_trajRef.d0-1);
  sRef = linspace(0.,1.,_trajRef.d0-1);

  if (useOrientation) {
    goalMO->setOrientation(goalRef.subRange(3,5));
  }

  traj = ~x0;
  s = ARR(0.);

  lastGoal = goalRef+(x0-_trajRef[0]);
  eps_goal = 0.1;

  //compute velocity of input trajectory
  resizeAs(dtrajRef,_trajRef);
  for (uint j=0; j < _trajRef.d0; j++) {
    for (uint i=0; i < _trajRef.d1; i++) {
      if (j==0) {
        dtrajRef(j,i) = (_trajRef(j+1,i)-_trajRef(j,i))/dt;
      } else if (j==(_trajRef.d0-1)) {
        dtrajRef(j,i) = (_trajRef(j,i)-_trajRef(j-1,i))/dt;
      } else {
        dtrajRef(j,i) = (_trajRef(j+1,i)-_trajRef(j-1,i))/(2*dt);
      }
    }
  }

  trajWrap = new Spline(sRef,_trajRef,2);
  trajRef = new Spline(sRef,_trajRef,2);
}

void Pfc::iterate(arr& _state)
{
  state = _state;
  goal = goalMO->position;
  if (useOrientation) {
    goal.append(goalMO->orientation);
  }

  // update phase variable s
  if (traj.d0>2) {

    double goalRatio = length(goalRef - trajRef->eval(s.last()))/length(lastGoal-state);
    double stateRatio = length(state - traj[traj.d0-2])/length(traj[traj.d0-1] - traj[traj.d0-2]);
    s.append(s(s.d0-1) + dsRef*goalRatio*stateRatio);
  } else {
    s.append(s(s.d0-1) + dsRef);
  }

  // warp trajectory
  traj[traj.d0-1] = state;
  warpTrajectory();
  lastGoal = goal;

  // compute next state
  traj.append(traj[traj.d0-1] + (dsRef*trajWrap->deval(s.last())));
}

void Pfc::warpTrajectory()
{
  arr stateDiff = state-trajWrap->eval(s.last());
  arr goalDiff = goal-lastGoal;
  trajWrap->transform(goalDiff, stateDiff, s.last());
}

void Pfc::computeIK(arr &q, arr &qd)
{
    arr W, yPos, JPos, yVec, JVec, yPos_target,yVec_target, y_target, Phi, PhiJ, yCol,JCol,costs;

    W.setDiag(1.,orsG->getJointStateDimension());  // W is equal the Id_n matrix
    W = W*w_reg;

    // Compute current task states
    orsG->kinematicsPos(yPos, orsG->getBodyByName("endeff")->index);
    orsG->jacobianPos(JPos, orsG->getBodyByName("endeff")->index);
    orsG->kinematicsVec(yVec, orsG->getBodyByName("endeff")->index);
    orsG->jacobianVec(JVec, orsG->getBodyByName("endeff")->index);

    // iterate pfc
    arr y = yPos;
    if (useOrientation) {
      y.append(yVec);
    }
    iterate(y);

    // next target
    y_target = traj[traj.d0-1];

    // task 1: POSITION
    yPos_target = y_target.subRange(0,2);
    costs = (yPos - yPos_target)/ fPos_deviation;
    posCosts.append(~costs*costs);
    Phi = ((yPos - yPos_target)/ fPos_deviation);
    PhiJ = (JPos / fPos_deviation);

    // task  2: ORIENTATION
    if (useOrientation) {
      yVec_target = y_target.subRange(3,5);
      costs = (yVec - yVec_target)/ fVec_deviation;
      vecCosts.append(~costs*costs);
      Phi.append(costs);
      PhiJ.append(JVec / fVec_deviation);
    }

    // task 3: COLLISION
    if (useCollAvoid) {
      orsG->phiCollision(yCol,JCol,0.15);
      costs = yCol / yCol_deviation;
      colCosts.append(~costs*costs);
      Phi.append(costs);
      PhiJ.append(JCol / yCol_deviation);
    }

    // compute joint updates
    qd = inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    q -= qd;
}

void Pfc::plotState()
{
  write(LIST<arr>(trajRef->points),"out/trajRef.output");
  write(LIST<arr>(trajWrap->points),"out/trajWrap.output");
  write(LIST<arr>(traj),"out/traj.output");

  gnuplot("set term wxt 1 title 'position 1'");
  gnuplot("plot 'out/trajRef.output' us 1,'out/trajWrap.output' us 1, 'out/traj.output' us 1");
  gnuplot("set term wxt 2 title 'position 2'");
  gnuplot("plot 'out/trajRef.output' us 2,'out/trajWrap.output' us 2, 'out/traj.output' us 2");
  gnuplot("set term wxt 3 title 'position 3'");
  gnuplot("plot 'out/trajRef.output' us 3,'out/trajWrap.output' us 3, 'out/traj.output' us 3");

  if (useOrientation) {
    gnuplot("set term wxt 4 title 'orientation 1'");
    gnuplot("plot 'out/trajRef.output' us 4,'out/trajWrap.output' us 4, 'out/traj.output' us 4");
    gnuplot("set term wxt 5 title 'orientation 2'");
    gnuplot("plot 'out/trajRef.output' us 5,'out/trajWrap.output' us 5, 'out/traj.output' us 5");
    gnuplot("set term wxt 6 title 'orientation 3'");
    gnuplot("plot 'out/trajRef.output' us 6,'out/trajWrap.output' us 6, 'out/traj.output' us 6");
  }

  write(LIST<arr>(s),"out/s.output");
  write(LIST<arr>(sRef),"out/sRef.output");
  gnuplot("set term wxt 7 title 'phase profile'");
  gnuplot("plot 'out/s.output' us 1, 'out/sRef.output' us 1");


  //compute velocity of trajectory
  arr dtraj;
  resizeAs(dtraj,traj);
  for (uint j=0; j < traj.d0; j++) {
    for (uint i=0; i < traj.d1; i++) {
      if (j==0) {
        dtraj(j,i) = (traj(j+1,i)-traj(j,i))/dt;
      } else if (j==(traj.d0-1)) {
        dtraj(j,i) = (traj(j,i)-traj(j-1,i))/dt;
      } else {
        dtraj(j,i) = (traj(j+1,i)-traj(j-1,i))/(2*dt);
      }
    }
  }
  write(LIST<arr>(sqrt(sum(sqr(~(~dtrajRef).subRange(0,2)),1)) ),"out/dtrajRef.output");
  write(LIST<arr>(sqrt(sum(sqr(~(~dtraj).subRange(0,2)),1)) ),"out/dtraj.output");
  gnuplot("set term wxt 11 title 'velocity profile'");
  gnuplot("plot 'out/dtrajRef.output' us 1,'out/dtraj.output' us 1");

  //plot costs
  gnuplot("set term wxt 21 title 'cost overview'");
  write(LIST<arr>(posCosts),"out/posCosts.output");
  gnuplot("plot 'out/posCosts.output' us 1");

  if (useOrientation) {
    write(LIST<arr>(vecCosts),"out/vecCosts.output");
     gnuplot("replot 'out/vecCosts.output' us 1");
  }
  if (useCollAvoid) {
    write(LIST<arr>(colCosts),"out/colCosts.output");
      gnuplot("replot 'out/colCosts.output' us 1");
  }
}

void Pfc::printState()
{
  cout << "TRef = " << TRef << endl;
  cout << "dt = " << dt << endl;
  cout << "goalRef = " << goalRef << endl;
  cout << "x0 = " << x0 << endl;
  cout << "dsRef = " << dsRef << endl;
}
