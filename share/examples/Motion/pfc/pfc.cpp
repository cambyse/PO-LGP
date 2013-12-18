#include "pfc.h"

Pfc::Pfc(ors::KinematicWorld &_orsG, arr& _trajRef, double _TRef, arr &_x0, arr &_q0, MObject &_goalMO, \
        bool _useOrientation, bool _useCollAvoid, \
        double _fPos_deviation, double _fVec_deviation, double _yCol_deviation, double _w_reg):
        fPos_deviation(_fPos_deviation),
        fVec_deviation(_fVec_deviation),
        yCol_deviation(_yCol_deviation),
        w_reg(_w_reg),
        TRef(_TRef),
        useOrientation(_useOrientation),
        useCollAvoid(_useCollAvoid),
        goalMO(&_goalMO),
        orsG(&_orsG),
        x0(_x0),
        q0(_q0)
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
  joints_bk.append(~q0);

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

    joints_bk.append(~q);

    // Compute current task states
    orsG->kinematicsPos(yPos, JPos, orsG->getBodyByName("endeff")->index);
    orsG->kinematicsVec(yVec, JVec, orsG->getBodyByName("endeff")->index);

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
      orsG->kinematicsProxyCost(yCol,JCol,0.15);
      costs = yCol / yCol_deviation;
      colCosts.append(~costs*costs);
      Phi.append(costs);
      PhiJ.append(JCol / yCol_deviation);
    }

    // compute joint updates
    qd = -inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    q += qd;


}

void Pfc::plotState()
{
  if (scene.M==0) {
    scene = STRING("out");
  }else {
    scene = STRING("out/"<<scene);
  }
  cout << "Save Path: " << scene << endl;

  write(LIST<arr>(joints_bk),STRING(scene<<"/joints_bk.output"));
  write(ARR(dt),STRING(scene<<"/dt.output"));
  write(LIST<arr>(goal),STRING(scene<<"/goal.output"));

  write(LIST<arr>(trajRef->points), STRING(scene<<"/trajRef.output"));
  write(LIST<arr>(trajWrap->points),STRING(scene<<"/trajWrap.output"));
  write(LIST<arr>(traj),STRING(scene<<"/traj.output"));

  gnuplot("set term wxt 1 title 'position 1'");
  gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 1,'"<<scene<<"/trajWrap.output' us 1, '"<<scene<<"/traj.output' us 1"));
  gnuplot("set term wxt 2 title 'position 2'");
  gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 2,'"<<scene<<"/trajWrap.output' us 2, '"<<scene<<"/traj.output' us 2"));
  gnuplot("set term wxt 3 title 'position 3'");
  gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 3,'"<<scene<<"/trajWrap.output' us 3, '"<<scene<<"/traj.output' us 3"));

  if (useOrientation) {
    gnuplot("set term wxt 4 title 'orientation 1'");
    gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 4,'"<<scene<<"/trajWrap.output' us 4, '"<<scene<<"/traj.output' us 4"));
    gnuplot("set term wxt 5 title 'orientation 2'");
    gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 5,'"<<scene<<"/trajWrap.output' us 5, '"<<scene<<"/traj.output' us 5"));
    gnuplot("set term wxt 6 title 'orientation 3'");
    gnuplot(STRING("plot '"<<scene<<"/trajRef.output' us 6,'"<<scene<<"/trajWrap.output' us 6, '"<<scene<<"/traj.output' us 6"));
  }

  write(LIST<arr>(s),STRING(scene<<"/s.output"));
  write(LIST<arr>(sRef),STRING(scene<<"/sRef.output"));
  gnuplot("set term wxt 7 title 'phase profile'");
  gnuplot(STRING("plot '"<<scene<<"/s.output' us 1, '"<<scene<<"/sRef.output' us 1"));


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
  write(LIST<arr>(sqrt(sum(sqr(~(~dtrajRef).subRange(0,2)),1)) ),STRING(scene<<"/dtrajRef.output"));
  write(LIST<arr>(sqrt(sum(sqr(~(~dtraj).subRange(0,2)),1)) ),STRING(scene<<"/dtraj.output"));
  gnuplot("set term wxt 11 title 'velocity profile'");
  gnuplot(STRING("plot '"<<scene<<"/dtrajRef.output' us 1,'"<<scene<<"/dtraj.output' us 1"));

  //plot costs
  gnuplot("set term wxt 21 title 'cost overview'");
  write(LIST<arr>(posCosts),STRING(scene<<"/posCosts.output"));
  gnuplot(STRING("plot '"<<scene<<"/posCosts.output' us 1"));

  if (useOrientation) {
    write(LIST<arr>(vecCosts),STRING(scene<<"/vecCosts.output"));
     gnuplot(STRING("replot '"<<scene<<"/vecCosts.output' us 1"));
  }
  if (useCollAvoid) {
    write(LIST<arr>(colCosts),STRING(scene<<"/colCosts.output"));
      gnuplot(STRING("replot '"<<scene<<"/colCosts.output' us 1"));
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
