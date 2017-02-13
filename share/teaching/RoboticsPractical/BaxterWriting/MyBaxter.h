#pragma once

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

#include <Control/TaskControlThread.h>
#include <Hardware/gamepad/gamepad.h>
#include <Kin/kinViewer.h>

#include <Motion/taskMap_qItself.h>

#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/JointCommand.h>

#include <Core/array.h>
#include <Core/graph.h>
#include <Core/util.h>
#include <Motion/taskMaps.h>

//#include <Media/audio.h>



baxter_core_msgs::JointCommand conv_qRef2baxterMessage(const arr& q_ref, const mlr::KinematicWorld& baxterModel, const char* prefix);

class MyBaxter {
    Access_typed<sensor_msgs::JointState> jointState;//(NULL, "jointState");
    ACCESSname(FilterObjects, object_database)

    TaskControlThread tcmBax;
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
    ros::Publisher pubVI_L;
    //ros::Publisher pubVI_R;

    CtrlTask *openG, *closeG;
    mlr::Array<CtrlTask*> set;

    mlr::KinematicWorld baxterModel;
    mlr::KinematicWorld testWorld;

public:
    float sec = 200.;
    Access_typed<arr> ctrl_q_ref;

    MyBaxter():
        jointState(NULL, "jointState"),
        tcmBax("baxter"),
        rosInit("MyBaxter"),
        ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcmBax.realWorld),
        ctrl_q_ref(NULL, "ctrl_q_ref"),
        spctb(tcmBax.realWorld),
        sub("/robot/joint_states", jointState){
        openG= new CtrlTask("openG",
              new TaskMap_qItself( tcmBax.realWorld, "l_gripper_l_finger_joint"),
                                     0.4, 1., 1., 1.);
        closeG= new CtrlTask("closeG",
              new TaskMap_qItself( tcmBax.realWorld, "l_gripper_l_finger_joint"),
                                     0.4, 1., 1., 1.);


        if(mlr::getParameter<bool>("useRos")){
          rosCheckInit("minimalPositionControl");
          new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
          nh = new ros::NodeHandle;
          pubVI_L = nh->advertise<baxter_core_msgs::JointCommand>("robot/limb/left/joint_command", 1);
          //pubVI_R = nh->advertise<baxter_core_msgs::JointCommand>("robot/limb/right/joint_command", 1);
        }
        threadOpenModules(true);
        tcmBax.verbose = false;
        testWorld = tcmBax.realWorld;
    }

    ~MyBaxter(){
      delete nh;
      threadCloseModules();
    }

    void homing();
    CtrlTask* align( char* name, char* frame1, mlr::Vector vec1, char* frame2, mlr::Vector vec2, double ref);
    CtrlTask* goToPosition(const char* frame1, const char* frame2, arr pos, double timeScale=1.0);
    CtrlTask* goToPositionTest(const char* frame1, const char* frame2, arr pos, double timeScale);
    void changePosition(CtrlTask *position, arr pos);
    void addTask(CtrlTask *task, int wait, int weight=100);
    void removeTask(CtrlTask *task);
    void stop(const CtrlTaskL& tasks);
    void closeGripper();
    void openGripper();


    void waitConv(const CtrlTaskL& tasks);
    arr q0();

    //-- get object information
    uint reportPerceptionObjects();
    void reportJointState();
    arr getEfforts();
    arr getJointState();
    void getState(arr& q, arr& qdot, arr& u);
    double setTestJointState(const arr& q);
    double setModelJointState(const arr &q);
    void getEquationOfMotion(arr& M, arr& F);


    //-- get position closest cluster
    mlr::Vector closestCluster();
    mlr::Vector arPose();

    void disablePosControlL();
    void disablePosControlR();
    void enablePosControlL();
    void enablePosControlR();

    void enableTotalTorqueModeL();
    void enableTotalTorqueModeR();
    void disableTotalTorqueModeL();
    void disableTotalTorqueModeR();
    void publishTorque(const arr& u, const char* prefix="left_");

    const mlr::KinematicWorld& getKinematicWorld();

    arr getJointLimits();
    double getCollisionScalar();

    //-- inner access
    struct TaskControlThread& getTaskControlThread();

    const mlr::KinematicWorld& getModelWorld();
    void sendJoints(const arr &q);

    //--writing
	// using inverse kinematics
    void writeLetterSim(ofstream& myfile, arr delta_y, arr nrSteps, arr force, CtrlTask &pos_eeL, arr q0);
    void writeLetterReal(arr delta_y, arr nrSteps, arr force, CtrlTask &pos_eeL);
	// using position control
    void writeLettersToFile(CtrlTask *pos, arr &y, arr delta_y, arr nrSteps, arr force, ofstream &myfile);  

    // uses data from file
	void writeLettersData(arr Data, CtrlTask &pos_eeL);
    
    void findTheTable(CtrlTask *pos_eeL, arr target);

	void useRos(bool uR);
};







