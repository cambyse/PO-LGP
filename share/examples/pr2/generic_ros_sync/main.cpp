#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>
#include <std_msgs/String.h>

//===========================================================================
// #define BEGIN_ROSMODULE(name, topic, msg_type) \
//   struct ROS_#name : Module { \
//     ros::NodeHandle _nh; \
//     ros::Subscriber _sub; \
//     \
//     virtual void open() { \
//       rosCheckInit(); \
//       this->_sub  = s->nh.subscribe(#TOPIC, 1, &RosSub_#TOPIC::callback, this); \
//     }; \
//     virtual void step() {}; \
//     virtual void close() { \
//       this->_nh.shutdown(); \
//     }; \
//     void callback(const #type::ConstPtr& msg) { \
//       this->#TOPIC.set() = msg; \
//     } \
//     TODO add ACCESS


// #define END_MODULE() };

const char* topic_name{"tmp_test"};

class ROSSub_XXX : public Module {
public:
  ROSSub_XXX() : Module(topic_name) { cout << "ctor" <<endl; }

  ACCESS(std_msgs::String, data)
  // struct __XXX__Access:Access_typed<std_msgs::String> {
  //   __XXX__Access():Access_typed<std_msgs::String>(){}
  // } _data;

  void open() {
    cout << "in open" << endl;
    rosCheckInit();
    this->_nh = new ros::NodeHandle;
    this->_sub  = this->_nh->subscribe(
        topic_name, 1,
        &ROSSub_XXX::callback, this);
  }
  void step() { cout << "step" << endl; }
  void close() {
    this->_nh->shutdown();
  }
  void callback(const std_msgs::String::ConstPtr& msg) {
    cout << *msg << endl;
    // this->_data.set() = msg;
  }

private:
  ros::NodeHandle* _nh;
  ros::Subscriber _sub;
};

// =================================================================================================
struct MySystem:System {
  // Access Variables
  // ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs)

  MySystem() {
    if (MT::getParameter<bool>("useRos", true)){
      cout << "using ROS" << endl;
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      // addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
      addModule<ROSSub_XXX>(NULL, Module::listenFirst, .1);
    }
    connect();
  }
};


// =================================================================================================
int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  MySystem system;
  engine().open(system);
  cout << "open " << endl;

  for (int i = 0; true; i++) {
    MT::wait(0.1);
  cout << "in loop" << endl;
  }

  engine().close(system);
  cout <<"bye bye" <<endl;
  return 0;
}
