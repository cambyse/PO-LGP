#include "MyBaxter.h"


void augmentDataWithF(mlr::String filename){
  arr D = FILE(filename);

  ors::KinematicWorld W("rawbaxter.ors");

  filename <<"_Faugmented";
  ofstream fil(filename);
  cout <<"output = '" <<filename <<"'" <<endl;

  for(uint i=0;i<D.d0;i++){
    if(!(i%10)) cout <<i <<endl;
    const arr& Di = D[i];
    arr q = Di.refRange(0,16);
    arr u = Di.refRange(17,-1);
    W.setJointState(q);
    W.gl().update();
    arr M,F;
    W.equationOfMotion(M, F);

    fil <<q <<' ' <<F <<' ' <<u <<endl;
  }
  fil.close();
}

void MyBaxter::getState(arr& q, arr& qdot, arr& u){
  for(;;){
    baxter_get_q_qdot_u(q, qdot, u, jointState.get(), tcmBax.realWorld);
    if(fabs(q(0))>1e-10) return;
    jointState.waitForNextRevision();
  }
}

void MyBaxter::getEquationOfMotion(arr& M, arr& F){
  testWorld.equationOfMotion(M, F);
}

arr MyBaxter::getJointLimits(){
  return tcmBax.realWorld.getLimits();
}

arr MyBaxter::getJointState(){
  return tcmBax.realWorld.q;
}

double MyBaxter::setTestJointState(const arr &q){
   testWorld.setJointState(q);
   arr y;
   testWorld.kinematicsProxyCost(y, NoArr);
   testWorld.gl("testWorld").update();
   return y.scalar();
}

double MyBaxter::setModelJointState(const arr &q){
   arr qdot = tcmBax.modelWorld.get()->qdot;
   tcmBax.modelWorld.set()->setJointState(q, qdot);

   arr y;
   tcmBax.modelWorld.set()->kinematicsProxyCost(y, NoArr);
   tcmBax.modelWorld.set()->gl("modelWorld").update();
   return y.scalar();
}

const ors::KinematicWorld& MyBaxter::getKinematicWorld(){
  return tcmBax.realWorld;
}

const ors::KinematicWorld& MyBaxter::getModelWorld(){
  return tcmBax.modelWorld.get();
}

arr MyBaxter::getEfforts(){
    return baxter_getEfforts(jointState.get(), tcmBax.realWorld);
}

void MyBaxter::waitConv(const CtrlTaskL& tasks){
  for(;;){
    mlr::wait(.03);
    bool allConv=true;
    for(CtrlTask *t:tasks) if(!t->isConverged()){ allConv=false; break; }
    if(allConv) return;
  }
}

void MyBaxter::publishTorque(const arr& u, const char* prefix){
  if(nh){
    //cout <<"SENDING TORQUES: " <<u <<endl;
    baxter_core_msgs::JointCommand msg = conv_qRef2baxterMessage(u, tcmBax.realWorld, prefix);
    msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
    pubVI_L.publish(msg);
  }
}

void MyBaxter::addTask(CtrlTask *task, int weight=100, int wait){
    task->prec=weight;
    set.append(task);
    tcmBax.ctrlTasks.set()=set;
    mlr::wait(wait);
}

void MyBaxter::removeTask(CtrlTask *task){
    set.removeValue(task);
    tcmBax.ctrlTasks.set()=set;
}

void MyBaxter::openGripper(){
    openG->y_ref= {0.1};
    set.append(openG);
    tcmBax.ctrlTasks.set()=set;
    waitConv({openG});
    //mlr::wait(1.);
    set.removeValue(openG);
    tcmBax.ctrlTasks.set()=set;
}

void MyBaxter::closeGripper(){
    closeG->y_ref= {0.0};
    set.append(closeG);
    tcmBax.ctrlTasks.set()=set;
    waitConv({closeG});
    //mlr::wait(1.);
    set.removeValue(closeG);
    tcmBax.ctrlTasks.set()=set;
}

void MyBaxter::homing(){
    //-- create a homing with
    CtrlTask *homing=new CtrlTask("homing",
                  new TaskMap_qItself(),
                  .5, 1., .2, 10.);
    homing->y_ref = tcmBax.q0;
    tcmBax.ctrlTasks.set() = { homing };
    waitConv({homing});
}

void MyBaxter::sendJoints(const arr& q){
    CtrlTask *joints=new CtrlTask("joints",
                  new TaskMap_qItself(),
                  2., 0.8, 1., 1.);
    joints->y_ref = q;
    tcmBax.ctrlTasks.set() = { joints };

}

CtrlTask* MyBaxter::goToPosition(const char* frame1, const char* frame2, arr pos, double timeScale){
    //--go to position
    CtrlTask *position = new CtrlTask(frame1, //name
        new DefaultTaskMap(posTMT, tcmBax.modelWorld.get()(), frame1, NoVector, frame2), //map
                         timeScale, 0.8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc

        position->map.phi(position->y, NoArr, tcmBax.modelWorld.get()()); //get the current value

   position->y_ref = pos; //set a target
   set.append(position);
   tcmBax.ctrlTasks.set()=set;

   waitConv({position});
   return position;
}

CtrlTask* MyBaxter::goToPositionTest(const char* frame1, const char* frame2, arr pos, double timeScale){
    //--go to position
    CtrlTask *position = new CtrlTask(frame1, //name
        new DefaultTaskMap(posTMT, testWorld, frame1, NoVector, frame2), //map
                         timeScale, 0.8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc

        position->map.phi(position->y, NoArr, testWorld); //get the current value

   position->y_ref = pos; //set a target
   set.append(position);
   tcmBax.ctrlTasks.set()=set;

   waitConv({position});
   return position;
}

//position.prec=100; weight

void MyBaxter::changePosition(CtrlTask* position, arr pos){
    set.removeValueSafe(position);
    tcmBax.ctrlTasks.set()=set;
    position->y_ref=pos;

    set.append(position);

    tcmBax.ctrlTasks.set()=set;
    waitConv(set);
}

CtrlTask* MyBaxter::align(char *name, char* frame1, ors::Vector vec1, char* frame2, ors::Vector vec2, double ref){
    CtrlTask *align = new CtrlTask(name,
        new DefaultTaskMap(vecAlignTMT, tcmBax.modelWorld.get()(), frame1, vec1, frame2, vec2),
                        1., .8, 1., 1.);
    align->y_ref = {ref};
    set.append(align);
    tcmBax.ctrlTasks.set()=set;
    return align;
}

uint MyBaxter::reportPerceptionObjects(){
  object_database.readAccess();
  FilterObjects clusters;
  uint n=0;
  for(FilterObject* fo : object_database()){
    fo->write(cout);
    cout <<endl;
    n++;
  }
  object_database.deAccess();
  return n;
}

ors::Vector MyBaxter::closestCluster(){
  object_database.readAccess();

  ors::Vector toReturn(0,0,0);

  double max_dist = DBL_MIN;
  for(FilterObject* fo : object_database())
  {
    if (fo->type == FilterObject::FilterObjectType::cluster)
    {
      ors::Vector mean = dynamic_cast<Cluster*>(fo)->transform.pos;
      double dist = dynamic_cast<Cluster*>(fo)->transform.pos.z;
      if (max_dist < dist)
      {
        max_dist = dist;
        toReturn = mean;
      }
    }
  }
  object_database.deAccess();

  return toReturn;
}

ors::Vector MyBaxter::arPose(){
  object_database.readAccess();

  ors::Vector toReturn(0,0,0);

  for(FilterObject* fo : object_database())
  {
    if ((fo->id == 2) && (fo->type == FilterObject::FilterObjectType::alvar))
    {
      ors::Transformation pos = fo->frame * fo->transform;
      toReturn = pos.pos;
      std::cout << toReturn << std::endl;
    }
  }
  object_database.deAccess();

  return toReturn;
}

arr MyBaxter::q0(){
  return tcmBax.q0;
}

void MyBaxter::disablePosControlR(){
  spctb.enablePositionControlR = false;
}
void MyBaxter::disablePosControlL(){
  spctb.enablePositionControlL = false;
}

void MyBaxter::enablePosControlR(){
  spctb.enablePositionControlR = true;
}
void MyBaxter::enablePosControlL(){
  spctb.enablePositionControlL = true;
}

void MyBaxter::enableTotalTorqueModeR(){
  spctb.totalTorqueModeR = true;
}
void MyBaxter::enableTotalTorqueModeL(){
  spctb.totalTorqueModeL = true;
}

void MyBaxter::disableTotalTorqueModeR(){
  spctb.totalTorqueModeR = false;
}
void MyBaxter::disableTotalTorqueModeL(){
  spctb.totalTorqueModeL = false;
}

void MyBaxter::reportJointState(){
  if(mlr::getParameter<bool>("useRos"))
  {
    sensor_msgs::JointState js = jointState.get();

    std::cout << "Joint header: " << js.header.seq << std::endl;
    for (uint i = 0; i < js.name.size(); ++i){
      std::cout << "\tJoint: " << js.name[i] << "\tPosition: " << js.position[i] << "\tEffort: " << js.effort[i] << std::endl;
    }
  }
}

double MyBaxter::getCollisionScalar(){
  arr y;
  tcmBax.modelWorld.get()->kinematicsProxyCost(y, NoArr);
  return y.scalar(); /* 0(colFREE)-1(col) -- 0.5(pref) */
}


void MyBaxter::stop(const CtrlTaskL& tasks){
  for(CtrlTask *t:tasks) set.removeValue(t);
  tcmBax.ctrlTasks.set() = set;
  for(CtrlTask *t:tasks){
    delete &t->map;
    delete t;
  }
}

// Writes the configuration q of letter(matrix of offsets) into a file - configuration q computed via inverse kinematics
void MyBaxter::writeLetterSim(ofstream& myfile, arr delta_y, arr nrSteps, arr force, CtrlTask &pos_eeL, arr q0){

    arr y, d_y, y_step, y_origin, y_target;
    arr J, pseudoJ;
    arr q_ref, delta_q;
    int press;


    q0 = testWorld.q;
    q_ref = q0;
    arr I = eye(q0.d0);
    for(uint i=0; i < delta_y.d0; i++){
        press = force(i);
        pos_eeL.map.phi(y, J, testWorld);
        y_origin = y;
        y_target = y + delta_y[i];

        for(double step = 0; step < nrSteps(i); step++){

            y_step = y_origin + (step/nrSteps(i)) * (y_target-y_origin);

            d_y = y_step - y;
            pseudoJ = ~J*inverse(J*~J);
            delta_q = pseudoJ * d_y;
            q_ref = q_ref + delta_q + 0.5*(I - pseudoJ*J) * (q0 - q_ref);

            myfile<< q_ref <<" "<< press <<endl;

            setTestJointState(q_ref);
            mlr::wait(0.05);

            pos_eeL.map.phi(y, J, testWorld);
        }
    }
}


// Writes the letter(matrix of offsets) on table - configuration q computed via inverse kinematics (not working)
void MyBaxter::writeLetterReal(arr delta_y, arr nrSteps, arr forceFlag, CtrlTask &pos_eeL){

    arr y, d_y, y_step, y_origin, y_target;
    arr J, pseudoJ, Jz, Jxy;
    arr q0, q_ref, delta_q, qdot, q;
    arr M, F, a, u;
    int press;
    double force = -2;
    double kp = mlr::getParameter<double>("kp");
    double kd = mlr::getParameter<double>("kd");


    getState(q0, qdot, u);
    //q0 = testWorld.q;

    q_ref = q0;
    arr I = eye(q0.d0);
    for(uint i=0; i < delta_y.d0; i++){
        press = forceFlag(i);
        //pos_eeL.map.phi(y, J, testWorld);
        pos_eeL.map.phi(y, J, getKinematicWorld());
        Jxy = J.sub(0,1,0,-1);
        Jz = J.sub(2,-1,0,-1);
        y_origin = y;
        y_target = y + delta_y[i];

        for(double step = 0; step < nrSteps(i); step++){

            y_step = y_origin + (step/nrSteps(i)) * (y_target-y_origin);

            d_y = y_step - y;
            pseudoJ = ~Jxy*inverse(Jxy*~Jxy);
            delta_q = pseudoJ * d_y.sub(0,1);
            //q_ref = q_ref + delta_q + (I - pseudoJ*J) * (q0 - q_ref);
            q_ref = q_ref + delta_q + (I - pseudoJ*Jxy) * (q0 - q_ref);

            for(uint i=0;i<0.5*sec;i++){
                cout<<"---------------------------------------"<<endl;
                getState(q, qdot, u);

                a = kp * (q_ref-q) - kd * qdot;

                //-- translate to motor torques
                getEquationOfMotion(M, F);

               //-- torques for the position (x and y) + torques for the force (z)
                if(press)
                    u = M*a + ~Jz * force;
                else
                    u = M*a;

                cout << "Torques published: " << u <<endl;

                publishTorque(u, "left_");
                mlr::wait(0.005);
                //setTestJointState(q_ref);
            }

            //pos_eeL.map.phi(y, J, testWorld);
            pos_eeL.map.phi(y, J, getKinematicWorld());
            Jz = J.sub(2,-1,0,-1);
            Jxy = J.sub(0,1,0,-1);
        }
    }
}


// Writes on the table; configurations q read from file
void MyBaxter::writeLettersData(arr Data, CtrlTask &pos_eeL){

    arr y,J;
    arr Jz;
    arr M, F, a;
    arr q, qdot, q_ref, u;
    int press;

    double force = -1.5;
    double kp = mlr::getParameter<double>("kp");
    double kd = mlr::getParameter<double>("kd");

    for(uint dim=0; dim<Data.d0; dim++){
        q_ref = Data[dim].sub(0,16);
        press = Data[dim](17);

        pos_eeL.map.phi(y, J, getKinematicWorld());
        Jz = J.sub(2,-1,0,-1);

        for (uint i = 0; i < 20; i++){
            getState(q, qdot, u);
            a = kp * (q_ref-q) - kd * qdot;

            //-- translate to motor torques
            getEquationOfMotion(M, F);

            //-- torques for the position (x and y) + torques for the force (z)
            if(press)
                u = M*a + ~Jz * force;
            else
                u = M*a;

            publishTorque(u, "left_");
            mlr::wait(0.005);
        }
    }
}

void MyBaxter::useRos(bool uR){ tcmBax.useRos = uR;}

//The endeffector goes down until it "feels" the table
void MyBaxter::findTheTable(CtrlTask *pos_eeL, arr target){

    double epsilon = 0.0001;
    double step = 0.001;
    arr y, J;
    arr q0, qdot, u, f;
    arr I=eye(3);

    while(1)
    {
          pos_eeL->map.phi(y, J, getKinematicWorld());

          getState(q0,qdot,u);
          f = ~(~J*inverse(J*~J + epsilon*I)) * u;
          cout<<"Force: "<< f <<endl;
          //baxter.jointState.waitForNextRevision();
          double reach = 30;
          if((f(2) > reach + 6) && target(2) > 0.7)
          {
              target = target - ARR(.0, .0, step);
              changePosition(pos_eeL, target);
              cout<<"Position: "<< y <<endl;
              reach= 63.714 * y(2)-28.8;
          }
          else
          {
              cout<<"Here is the table!"<<endl;
              target = target + ARR(.0, .0, 10*step);
              changePosition(pos_eeL, target);
              mlr::wait(3.);
              break;
          }
    }
}

// Writes the configuration q of letter(matrix of offsets) into a file - we get configuration q through position control
void MyBaxter::writeLettersToFile(CtrlTask *pos, arr &y, arr delta_y, arr nrSteps, arr force, ofstream &myfile){
arr y_new, y_step, q;
    for(uint i=0; i<delta_y.d0;i++)
    {
        for(double j = 0; j<=nrSteps(i); j++)
        {
            y_step = j*(delta_y[i]/nrSteps(i));
            y_new = y + y_step;
            changePosition(pos, y_new);
            mlr::wait(0.5);
            q = getModelWorld().q;
            myfile << q <<" "<<force(i)<<endl;
        }
        y = y + delta_y[i];
    }
}
