#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>
#include <std_msgs/String.h>

//===========================================================================
#define ROSSUB(topic_name, msg_type, var_name) \
  class ROSSUB_##var_name : public Module { \
  private: \
    ros::NodeHandle* _nh; \
    ros::Subscriber _sub; \
    \
  public: \
    ACCESS(msg_type, var_name) \
    ROSSUB_##var_name() : Module(#var_name) {} \
    void open() { \
      rosCheckInit(); \
      this->_nh = new ros::NodeHandle; \
      this->_sub  = this->_nh->subscribe( \
        topic_name, 1, &ROSSUB_##var_name::callback, this); \
    } \
    void step() {} \
    void close() { \
      this->_nh->shutdown(); \
      delete _nh; \
    } \
    void callback(const msg_type::ConstPtr& msg) { \
      this->var_name.set() = *msg; \
    } \
  };

//===========================================================================
ROSSUB("/hello_world", std_msgs::String, hello_world);
// --> ROSSUB_hello_world


// =================================================================================================
struct MySystem:System {
  // Access Variables
  ACCESS(std_msgs::String, hello_world)

  MySystem() {
    if (MT::getParameter<bool>("useRos", true)){
      cout << "using ROS" << endl;
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<ROSSUB_hello_world>(NULL, Module::listenFirst, .1);
    }
    connect();
  }
};


// =================================================================================================
int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  MySystem system;
  engine().open(system);
  cout << "Engine open " << endl;

  for (int i = 0; true; i++) {
    system.hello_world.waitForNextRevision();
    std_msgs::String d = system.hello_world.get();
    cout << d << endl;
  }

  engine().close(system);
  cout <<"bye bye" <<endl;
  return 0;
}
