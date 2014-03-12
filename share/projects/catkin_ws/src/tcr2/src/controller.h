#include "tcr2/SetGoal.h"

#include <Ors/ors.h>
#include <Gui/opengl.h>

namespace ros {
  class NodeHandle;  
}

struct OpenGL;

class Controller {
  public:
    Controller(ros::NodeHandle& n, const char* endeff);
    void run();  
    void step();

    bool set_goal(tcr2::SetGoal::Request &req,
                  tcr2::SetGoal::Response &res);

  private:
    ros::NodeHandle& n;

    OpenGL gl;
    ors::KinematicWorld G;
    ors::Vector goal;

    // Controller gains
    double Kp, Kd, Ki;

    // integration for i-part
    ors::Vector integrate;

    const char* endeff;
};
