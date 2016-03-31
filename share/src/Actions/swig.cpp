#include "swig.h"

#include <FOL/fol.h>
#include <Ors/ors.h>
#include <Control/TaskControllerModule.h>
#include "ActivitySpinnerModule.h"
#include <Actions/RelationalMachineModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <RosCom/rosalvar.h>
#include <RosCom/roscom.h>
#include <RosCom/serviceRAP.h>
#include <Gui/opengl.h>
#include <csignal>
#include <Perception/perception.h>
#include <Perception/kinect2pointCloud.h>
#include <Ors/orsviewer.h>

// ============================================================================
struct SwigSystem* _g_swig;

struct SwigSystem {
  ACCESSname(ActivityL, A)
  ACCESSname(bool, quitSignal)
  ACCESSname(bool, fixBase)
  ACCESSname(RelationalMachine, RM)
  ACCESSname(mlr::String, effects)
  ACCESSname(mlr::String, state)
  ACCESSname(ors::KinematicWorld, modelWorld)
  ACCESSname(AlvarMarkers, ar_pose_markers)
  ACCESSname(visualization_msgs::MarkerArray, perceptionObjects)
  ACCESSname(arr, pr2_odom)
  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)

  ACCESSname(int, stopWaiting)
  ACCESSname(int, waiters)

  ACCESSname(byteA, kinect_rgb)
  ACCESSname(uint16A, kinect_depth)
  ACCESSname(ors::Transformation, kinect_frame)

  ACCESSname(arr, wrenchL)
  ACCESSname(arr, wrenchR)

  ACCESSname(byteA, modelCameraView)
  ACCESSname(byteA, modelDepthView)

  ACCESSname(arr, gamepadState)

  TaskControllerModule tcm;
  RelationalMachineModule rmm;
  OrsViewer orsviewer;
  OrsPoseViewer controlview;
  ActivitySpinnerModule aspin;
  GamepadInterface gamepad;

  RosCom_Spinner spinner;
  ServiceRAP rapservice;


//  PerceptionObjects2Ors percObjs;
//  ImageViewer camview;
//  Kinect2PointCloud k2pcl;
//  PointCloudViewer pclv;
//  AlvarSyncer alvar_syncer;

  Log _log;

  SwigSystem()
    : controlview({"ctrl_q_real", "ctrl_q_ref"}), /*camview("modelDepthView"),*/
      spinner("SwigSystem"), _log("SwigSystem"){

    computeMeshNormals(tcm.realWorld.shapes);
    controlview.setWorld(tcm.realWorld);
    if(mlr::getParameter<bool>("useRos",false)){
      cout <<"*** USING ROS" <<endl;
      rosCheckInit("SwigSystem");
      //addModule<ROSSUB_ar_pose_marker>(NULL, /*Module::loopWithBeat,*/ 0.05);
      //addModule<ROSSUB_perceptionObjects>(NULL, /*Module::loopWithBeat,*/ 0.02);
      // addModule<RosCom_ForceSensorSync>(NULL, /*Module::loopWithBeat,*/ 1.);

      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
      new Subscriber<AlvarMarkers>("/ar_pose_marker", (Access_typed<AlvarMarkers>&)ar_pose_markers);
      new SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>("/robot_pose_ekf/odom_combined", pr2_odom);
      new Subscriber<visualization_msgs::MarkerArray>("/tabletop/clusters", perceptionObjects);

      new SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA>("/kinect_head/rgb/image_color", kinect_rgb);
      new SubscriberConv<sensor_msgs::Image, uint16A, &conv_image2uint16A>("/kinect_head/depth/image_raw", kinect_depth, &kinect_frame);

//      new SubscriberConv<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr>("/ft_sensor/l_ft_compensated", wrenchL);
//      new SubscriberConv<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr>("/ft_sensor/r_ft_compensated", wrenchR);
      new SubscriberConv<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr>("/ft/l_gripper_motor", wrenchL);
      new SubscriberConv<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr>("/ft/r_gripper_motor", wrenchR);

    }else{
//      rosCheckInit("SwigSystem");
//      new RAP_roscom(rmm);
    }

    // make the base movable
    fixBase.set() = mlr::getParameter<bool>("fixBase", false);

    stopWaiting.set() = 0;
    waiters.set() = 0;

    _g_swig = this; //MT: what is _g_swig??

    //cout <<"SYSTEM=" <<registry() <<endl;
  }
};


void signal_catch(int signal) {
  cout << "Waiters: " <<_g_swig->waiters.get() << endl;
  _g_swig->stopWaiting.set() = _g_swig->waiters.get();
  _g_swig->effects.set()() << "stop, ";
  _g_swig->effects.set()() << "stop!, ";
  cout << "Ctrl-C pressed, try to stop all facts." << endl;
  raise(SIGABRT);
}
// ============================================================================

mlr::String lits2str(const stringV& literals, const dict& parameters=dict()){
  mlr::String str;
  str <<'(';
  for(auto& i:literals) str <<' ' <<i;
  str <<')';
  if(parameters.size()){
    str <<'{';
    for(auto& p:parameters) str <<' ' <<p.first <<'=' <<p.second;
    str <<'}';
  }
  return str;
}

// ============================================================================
// ActionSwigInterface

ActionSwigInterface::ActionSwigInterface(bool setSignalHandler): S(new SwigSystem){
  S->tcm.verbose=false;

  signal(SIGINT, signal_catch); //overwrite signal handler

  threadOpenModules(true, setSignalHandler);

  createNewSymbol("conv");
  createNewSymbol("contact");
  createNewSymbol("timeout");
  createNewSymbol("go");

  S->LOG(1) <<"Activity Symbols:";
  NodeL acts = registry().getNodes("Activity");
  for(Node *n:acts){
    S->LOG(1) <<"  adding symbol for " <<n->keys(0);
    createNewSymbol(n->keys.last().p);
  }
  S->LOG(1) <<"Shape Symbols:";
  S->modelWorld.writeAccess();
  for(ors::Shape *sh:S->modelWorld().shapes){
    S->LOG(1) <<"  adding symbol for Shape " <<sh->name;
    createNewSymbol(sh->name.p);
  }
  S->modelWorld.deAccess();
}


ActionSwigInterface::~ActionSwigInterface(){
  threadCloseModules();
  delete S;
  S=NULL;
}

void ActionSwigInterface::setVerbose(bool verbose) {
  S->tcm.verbose = verbose;
}

void ActionSwigInterface::setFixBase(bool base) {
  S->fixBase.set() = base;
}

void ActionSwigInterface::Cancel(){
  //engine().cancel(*S);
  cout << S->quitSignal.get();
  S->quitSignal.set() = true;
}

stringV ActionSwigInterface::getShapeList(){
  stringV strs;
  std::stringstream tmp;
  S->modelWorld.readAccess();
  for(ors::Shape *shape: S->modelWorld().shapes){
    tmp.str(""),
    tmp.clear();
    tmp <<shape->name;
    strs.push_back(tmp.str());
  }
  S->modelWorld.deAccess();
  return strs;
}

stringV ActionSwigInterface::getBodyList(){
  stringV strs;
  std::stringstream tmp;
  S->modelWorld.readAccess();
  for(ors::Body *body: S->modelWorld().bodies){
    tmp.str(""),
    tmp.clear();
    tmp << body->name;
    strs.push_back(tmp.str());
  }
  S->modelWorld.deAccess();
  return strs;
}

stringV ActionSwigInterface::getJointList(){
  stringV strs;
  std::stringstream tmp;
  S->modelWorld.readAccess();
  for(ors::Joint *joint: S->modelWorld().joints){
    tmp.str(""),
    tmp.clear();
    tmp << joint->name;
    strs.push_back(tmp.str());
  }
  S->modelWorld.deAccess();
  return strs;
}


dict ActionSwigInterface::getBodyByName(std::string bodyName){
  dict D;
  S->modelWorld.readAccess();
  ors::Body *body = S->modelWorld().getBodyByName(bodyName.c_str());
  D["name"]= bodyName;
  D["type"] = std::to_string(body->type);
  D["Q"] =  STRING('[' <<body->X.rot<<']');
  D["X"] = STRING('[' <<body->X.rot.getX()<< ']');
  D["Y"] = STRING('[' <<body->X.rot.getY()<< ']');
  D["Z"] = STRING('[' <<body->X.rot.getZ()<< ']');

  D["pos"] = STRING('[' <<body->X.pos<<']');
  S->modelWorld.deAccess();
  return D;
}

dict ActionSwigInterface::getJointByName(std::string jointName){
  dict D;
  S->modelWorld.readAccess();
  ors::Joint *joint = S->modelWorld().getJointByName(jointName.c_str());
  D["name"]= jointName;
  D["type"] = std::to_string(joint->type);
  D["Q"] =  STRING('[' <<joint->X.rot<<']');
  D["X"] = STRING('[' <<joint->X.rot.getX()<< ']');
  D["Y"] = STRING('[' <<joint->X.rot.getY()<< ']');
  D["Z"] = STRING('[' <<joint->X.rot.getZ()<< ']');
  D["pos"] = STRING('[' <<joint->X.pos<<']');

  arr q;
  S->modelWorld().getJointState(q);
  arr qj(joint->qDim());
  for (uint i=0;i<joint->qDim();i++) {
     qj(i) = q(joint->qIndex+i);
  }
  D["q"] = STRING(qj);
  D["axis"] = STRING('[' << joint->axis << ']');
  S->modelWorld.deAccess();
  return D;
}

dict ActionSwigInterface::getShapeByName(std::string shapeName){
  dict D;
  S->modelWorld.readAccess();
  ors::Shape *shape = S->modelWorld().getShapeByName(shapeName.c_str());
  D["name"]= shapeName;
  D["type"] = std::to_string(shape->type);
  D["Q"] =  STRING('[' <<shape->X.rot<<']');
  D["X"] = STRING('[' <<shape->X.rot.getX()<< ']');
  D["Y"] = STRING('[' <<shape->X.rot.getY()<< ']');
  D["Z"] = STRING('[' <<shape->X.rot.getZ()<< ']');
  D["phi"] = STRING('[' <<shape->X.rot.getX().phi()<< ']');
  D["theta"] = STRING('[' <<shape->X.rot.getX().theta()<< ']');

  D["pos"] = STRING('[' <<shape->X.pos<<']');
  S->modelWorld.deAccess();
  return D;
}


doubleV ActionSwigInterface::getQ() {
  arr q = S->modelWorld.get()->getJointState();
  return conv_arr2stdvec(q);
}

doubleV ActionSwigInterface::getV() {
  arr q, qdot;
  S->modelWorld.get()->getJointState(q, qdot);
  return conv_arr2stdvec(qdot);
}

double ActionSwigInterface::getQDim() {
  int qdim = S->modelWorld.get()->getJointStateDimension();
  return qdim;
}

int ActionSwigInterface::getSymbolInteger(std::string symbolName){
  Node *symbol = S->RM.get()->KB[symbolName.c_str()];
  CHECK(symbol,"The symbol name '" <<symbolName <<"' is not defined");
  return symbol->index;
}

intV ActionSwigInterface::str2lit(stringV symbolNames){
  intV I;
  for(auto& s:symbolNames) I.push_back(getSymbolInteger(s));
  return I;
}

stringV ActionSwigInterface::lit2str(intV literals){
  stringV strs;
  for(auto i:literals) strs.push_back(S->RM.get()->KB.elem(i)->keys.last().p);
  return strs;
}

bool  ActionSwigInterface::isTrue(const stringV& literals){
  return S->RM.get()->queryCondition(lits2str(literals));
}

void ActionSwigInterface::setFact(const char* fact){
  S->effects.set()() <<fact <<", ";
  S->state.waitForNextRevision(); //TODO: is this robust?
}

void ActionSwigInterface::stopFact(const char* fact){
  S->effects.set()() <<fact <<"!, ";
  S->state.waitForNextRevision(); //TODO: is this robust?
}

stringV ActionSwigInterface::getFacts(){
  stringV V;
  std::stringstream tmp;
  for(auto i:*S->RM.set()->state){
    tmp.str(""),
    tmp.clear();
    tmp << *i;
    V.push_back(tmp.str());

  }
  return V;
}

void ActionSwigInterface::startActivity(const stringV& literals, const dict& parameters){
  setFact(lits2str(literals, parameters));
}

void ActionSwigInterface::stopActivity(const stringV& literals){
  stopFact(lits2str(literals));
}

void ActionSwigInterface::waitForCondition(const stringV& literals){
  S->waiters.set()++;
  for(;;){
    if(isTrue(literals)) {
      S->waiters.set()--;
      return;
    } 
    if(S->stopWaiting.get() > 0) {
      S->stopWaiting.set()--;
      S->waiters.set()--;
      return;
    }
    S->state.waitForNextRevision();
  }
  // this->stopFact(literals);
}

void ActionSwigInterface::waitForCondition(const char* query){
  S->waiters.set()++;
  for(;;){
    if(S->RM.get()->queryCondition(query)){
      S->waiters.set()--;
      return;  
    }
    if(S->stopWaiting.get() > 0) {
      S->stopWaiting.set()--;
      S->waiters.set()--;
      return;
    }
    S->state.waitForNextRevision();
  }
  // this->stopFact(query);
}

int ActionSwigInterface::waitForOrCondition(const std::vector<stringV> literals){
  S->waiters.set()++;
  for(;;){
    for(unsigned i=0; i < literals.size(); i++){
      if(isTrue(literals[i])) {
        S->waiters.set()--;
        return i;
      }
      if(S->stopWaiting.get() > 0) {
        S->stopWaiting.set()--;
        S->waiters.set()--;
        return -1;   
      }
    }
    S->state.waitForNextRevision();
  }
  // this->stopFact(literals);
}

void ActionSwigInterface::waitForAllCondition(const stringV queries){
  S->waiters.set()++;
  for(;;){
    bool allTrue = true;
    for(unsigned i=0; i < queries.size(); i++){
      if(not S->RM.get()->queryCondition(mlr::String(queries[i]))) {
        allTrue = false;
      }
    }
    if(allTrue) {
      S->waiters.set()--;
      return;  
    }
    if(S->stopWaiting.get() > 0) {
      S->stopWaiting.set()--;
      S->waiters.set()--;
      return;
    }
    S->state.waitForNextRevision();
  }
  // this->stopFact(literals);
}

void ActionSwigInterface::waitForQuitSymbol(){
  waitForCondition(stringV({"quit"}));
}

int ActionSwigInterface::createNewSymbol(std::string symbolName){
#if 1
  Node *symbol = S->RM.set()->declareNewSymbol(symbolName.c_str());
#else
  Item *symbol = S->RM.set()->append<bool>(symbolName.c_str(), NULL, false);
#endif
  return symbol->index;
}

stringV ActionSwigInterface::getSymbols() {
  stringV V;
  std::stringstream tmp;
  for (auto* i:S->RM.get()->KB){
    tmp.str(""),
    tmp.clear();
    tmp <<i->keys.last(); // *i;//->keys(0);
    V.push_back(tmp.str());
  }
  return V;
}

int ActionSwigInterface::defineNewTaskSpaceControlAction(std::string symbolName, const stringV& parentSymbols, const dict& parameters){
#if 1
  mlr::String str;
  str <<symbolName.c_str() <<lits2str(parentSymbols, parameters);
  Node *symbol = S->RM.set()->declareNewSymbol(str);
#else
  S->RM.writeAccess();

  Item *symbol = S->RM().append<bool>(symbolName.c_str(), NULL, false);
  
  Graph& td = S->RM().appendSubgraph({"Task"}, {symbol})->value;
  td = parameters;
  S->RM().checkConsistency();
  //cout <<S->RM() <<endl;
  S->RM.deAccess();
  s->activity.machine->parseTaskDescription(*td);
#endif
  return symbol->index;
}

int ActionSwigInterface::getQIndex(std::string jointName) {
  return S->tcm.modelWorld.get()->getJointByName(mlr::String(jointName))->qIndex;
}

Access_typed<RelationalMachine>& ActionSwigInterface::getRM(){ return S->RM; }

void ActionSwigInterface::execScript(const char* filename){
  FILE(filename) >>S->RM.set()->KB;

  Node *s = S->RM.get()->KB["Script"];
  Graph& script = s->graph();
  int rev=0;
  for(Node* n:script){
    if(n->parents.N==0 && n->isGraph()){ //interpret as wait
      for(;;){
        if(allFactsHaveEqualsInScope(*S->RM.get()->state, n->graph())) break;
        rev=S->RM.waitForRevisionGreaterThan(rev);
      }
    }else{ //interpret as set fact
//      applySubstitutedLiteral(*S->RM.set()->state, n, {}, NULL);
      S->RM.set()->applyEffect(n, true);
//      S->rmm->threadStep();
//      S->effects.set()() <<"(go)"; //just trigger that the RM module steps
    }
  }
}

ors::Transformation ActionSwigInterface::getFramePose(const std::string& frame_id) {
  ors::Transformation frame = S->modelWorld.get()->getShapeByName(frame_id.c_str())->X;
  return frame;
}
