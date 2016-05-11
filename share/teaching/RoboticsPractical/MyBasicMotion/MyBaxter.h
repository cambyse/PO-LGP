#include <RosCom/serviceRAP.h>
#include <RosCom/baxter.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>

#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/subscribeTabletop.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>
#include <RosCom/publishDatabase.h>

#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>

#include <Motion/taskMap_qItself.h>

#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/JointCommand.h>

#include <Core/array.h>
#include <Core/graph.h>
#include <Motion/taskMaps.h>



class MyBaxter {
    Access_typed<sensor_msgs::JointState> jointState;//(NULL, "jointState");
    ACCESSname(FilterObjects, object_database)

    TaskControllerModule tcmBax;
    RosInit rosInit;    

    SubscribeTabletop tabletop_subscriber;
    SubscribeAlvar alvar_subscriber;
    Collector data_collector;
    Filter myFilter;
    PublishDatabase myPublisher;    

    GamepadInterface gamepad;
    OrsViewer view;
    OrsPoseViewer ctrlView;
    SendPositionCommandsToBaxter spctb;
    Subscriber<sensor_msgs::JointState> sub;

    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages
    ros::NodeHandle* nh;
    ros::Publisher pubVI_R;

    CtrlTask *openG, *closeG;
    mlr::Array<CtrlTask*> set;

    ors::KinematicWorld baxterModel;// !!!!

public:
    Access_typed<arr> ctrl_q_ref;

    MyBaxter():
        jointState(NULL, "jointState"),
        tcmBax("baxter"),
        rosInit("MyBaxter"),
        ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcmBax.realWorld),
        ctrl_q_ref(NULL, "ctrl_q_ref"),
        sub("/robot/joint_states", jointState){
        openG= new CtrlTask("openG",
              new TaskMap_qItself( tcmBax.realWorld, "l_gripper_l_finger_joint"),
                                     0.4, 1., 1., 1.);
        closeG= new CtrlTask("closeG",
              new TaskMap_qItself( tcmBax.realWorld, "l_gripper_l_finger_joint"),
                                     0.4, 1., 1., 1.);


        if(mlr::getParameter<bool>("useRos")){
          rosCheckInit("minimalPositionControl");
          //new SendPositionCommandsToBaxter();
          new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
          nh = new ros::NodeHandle;
          pubVI_R = nh->advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
        }
        threadOpenModules(true);
        tcmBax.verbose = true;
    }

    ~MyBaxter(){
      delete nh;
      threadCloseModules();
    }


    void homing();
    CtrlTask* goToPosition(const char* frame1, const char* frame2, arr pos, double timeScale=1.0);
    void changePosition(CtrlTask *position, arr pos);
    CtrlTask* align( char* name, char* frame1, ors::Vector vec1, char* frame2, ors::Vector vec2, double ref);
    void addTask(CtrlTask *task, int wait, int weight=100);
    void removeTask(CtrlTask *task);
    void closeGripper();
    void openGripper();
    void publishTorque(arr command);
    void waitConv(const CtrlTaskL& tasks);
    arr q0();
    uint reportPerceptionObjects();
    ors::Vector closestCluster();    
    ors::Vector arPose();
    void reportJointState();
    void disablePosControl();
    void enablePosControl();


};


void MyBaxter::waitConv(const CtrlTaskL& tasks){
  for(;;){
    mlr::wait(.03);
    bool allConv=true;
    for(CtrlTask *t:tasks) if(!t->isConverged()){ allConv=false; break; }
    if(allConv) return;
  }
}

baxter_core_msgs::JointCommand conv_qRef2baxterMessage(/*const arr& q_ref,*/ const ors::KinematicWorld& baxterModel, const char* prefix){
  baxter_core_msgs::JointCommand msg;
  //msg.mode = 1;
  msg.mode = 1;
  //for(ors::Joint *j:baxterModel.joints) if(j->name.startsWith(prefix)){
    //msg.command.push_back(q_ref(j->qIndex));
   // msg.command.push_back(0);
    //msg.names.push_back("right_s0");

    msg.command = {0};
    msg.names = {"right_s0"};
 // }
  return msg;
}

void MyBaxter::publishTorque(arr command)
{
  if(mlr::getParameter<bool>("useRos")){
    baxter_core_msgs::JointCommand msg;
    msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
    msg.names = { "right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2" };
    if (command.N != 7)
    {
      std::cout << "Incorrect command length: " << command.N << "!=7" << std::endl;
      exit(-1);
    }
    msg.command = { command(0), command(1), command(2), command(3), command(4), command(5), command(6) };
    pubVI_R.publish(msg);
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
    set.removeValue(openG);
    tcmBax.ctrlTasks.set()=set;
}

void MyBaxter::closeGripper(){
    closeG->y_ref= {0.0};
    set.append(closeG);
    tcmBax.ctrlTasks.set()=set;
    waitConv({closeG});
    set.removeValue(closeG);
    tcmBax.ctrlTasks.set()=set;
}

void MyBaxter::homing(){
    //-- create a homing with
    CtrlTask homing("homing",
                  new TaskMap_qItself(),
                  .5, 1., .2, 10.);
    homing.y_ref = tcmBax.q0;
    tcmBax.ctrlTasks.set() = { &homing };
    waitConv({&homing});
}

CtrlTask* MyBaxter::goToPosition(const char* frame1, const char* frame2, arr pos, double timeScale){
    //--go to position
    CtrlTask *position = new CtrlTask("endeffL", //name
        new DefaultTaskMap(posTMT, tcmBax.modelWorld.get()(), frame1, NoVector, frame2), //map
                         timeScale, 0.8, 1., 1.); //time-scale, damping-ratio, maxVel, maxAcc

        position->map.phi(position->y, NoArr, tcmBax.modelWorld.get()()); //get the current value

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
        //new DefaultTaskMap(vecAlignTMT, tcm.modelWorld.get()(), "endeffL", Vector_x, "shape1", Vector_z),
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

void MyBaxter::disablePosControl(){
  spctb.enable = false;
}

void MyBaxter::enablePosControl(){
  spctb.enable = true;
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
