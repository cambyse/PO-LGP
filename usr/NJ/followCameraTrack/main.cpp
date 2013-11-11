#define MT_IMPLEMENTATION

#include <signal.h>
#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/robot.h>


ofstream positions_file("positions.dat");

struct MyDemo:public TaskAbstraction {
  arr q;
  uint q_index, q_ind2;
  uint mult;//how much we stretch trajectory
  bool started_trajectory;


  virtual void updateTaskVariables(ControllerModule *ctrl); //overloading the virtual
  void init(RobotProcessGroup *robotProcesses);

  void loadPlainTrajectory(const char* filename);
  void followTrajectory(RobotProcessGroup *robotProcesses);
};




void MyDemo::init(RobotProcessGroup *robotProcesses){
  cout << "init TV_q = "<<TV_q->y << endl;
  cout << "init TV_x->x="<<TV_eff->y << endl;
  MT::IOraw = true;
  //if(robotProcesses->gui){
  robotProcesses->gui.gl->camera.setPosition(3.5,-8.,2.8);  // position of camera
  robotProcesses->gui.gl->camera.focus(0., -0.5, 1.);  // rotate the frame to focus the point (x,y,z)
  //}
  started_trajectory = false;
}


void MyDemo::updateTaskVariables(ControllerModule *ctrl){
  activateAll(TVall,false); //deactivate all variables
  TV_lim->active = true;  TV_lim->y_prec = 1e1;
  TV_col->active = true;  TV_col->y_prec = 1e-1;

  if (!started_trajectory && maxDiff(ctrl->q_home, TV_q->y)<0.001) {
    cout<<"Starting trajectory."<<endl;
    started_trajectory = true;
  }

  if (started_trajectory) {
    TV_q->active = true;
    TV_q->y_prec = 1e1;
    //TV_q->v_prec = 0.;
    TV_q->y_target = q[q_index]*((double)mult-q_ind2)/(double)mult + q[q_index+1]*(double)q_ind2/(double)mult;

    TV_q->v_prec= 1e-1; //8.;//5
    TV_q->v_target.setZero(); //damping on joint velocities
    q_ind2++;
    if(q_ind2 == mult){q_index++; q_ind2 = 0;}
  }else{ //homing
    TV_q->active = true;
    TV_q->y_prec = 0.;
    TV_q->v_prec = 1.;
    TV_q->v_target = ctrl->q_home - TV_q->y;
    double vmax = .3, v=norm(TV_q->v_target);
    if (v>vmax) TV_q->v_target *= vmax/v;
  }
}


void MyDemo::followTrajectory(RobotProcessGroup *robotProcesses){
  MT::String s6 = String("caliData");
  ofstream f6(s6);
  PerceptionModule perc;
  perc.threadOpen();

  for(;!robotProcesses->signalStop || (q_index == q.d0);){
    if (q_index%100 == 1 && q_ind2 == 0) {cout<< endl << endl << endl << q_index << "  of " << q.d0 << "err " << norm(robotProcesses->ctrl.q_reference-TV_q->y_target) << endl;}
    positions_file <<TV_eff->y <<endl;
    robotProcesses->evis.lock.readLock();  perc.lock.writeLock();
    perc.hsvChannelsL = robotProcesses->evis.hsvThetaL;
    perc.hsvChannelsR = robotProcesses->evis.hsvThetaR;
    robotProcesses->evis.lock.unlock();    perc.lock.unlock();
    robotProcesses->step();//calls the update task variables
    perc.threadStepOrSkip(0);

    //evis, perc -> gui
    robotProcesses->gui.lock.writeLock();  perc.lock.readLock();
    robotProcesses->gui.img[1] = robotProcesses->evis.cameraL;
    robotProcesses->gui.img[3] = perc.disp;
    listCopy(robotProcesses->gui.objects, perc.objects);
    //realizeObjectListInOrs(*robotProcesses->gui.ors, perc.objects);
    //copyBodyInfos(*robotProcesses->gui.ors2, *robotProcesses->gui.ors);
    robotProcesses->gui.lock.unlock();  perc.lock.unlock();

    if(perc.objects.N > 0){
      perc.lock.readLock();
      arr vision(4);
      for(uint i = 0; i < 4; i++)if(i < 2)
        vision(i) = perc.objects(0)->shapePointsL(0,i);
      else
        vision(i) = perc.objects(0)->shapePointsR(0,i-2);
      perc.lock.unlock();

      MT::IOraw=true;
      f6 <<  vision << " " << robotProcesses->ctrl.q_reference << " " <<   robotProcesses->ctrl.ors.getShapeByName("tennisBall")->X.pos << ' ' <<robotProcesses->stepCounter <<' ' <<MT::realTime() <<endl;
    }
    if(robotProcesses->joy.state(0)==16 || robotProcesses->joy.state(0)==32) break;
    if (q_index == q.d0-2) {
      break;
      started_trajectory = false;
      q_index = 0;
      q_ind2 = 0;
      cout<<"Random movements finished: Need to rehome."<<endl;
    }
  }
}

void MyDemo::loadPlainTrajectory(const char* filename){
  ifstream fil;
  MT::open(fil,filename);
  q.read(fil);cout << "Q dimensions " << q.d0 << " " << q.d1 << endl;
  q = q.sub(0,q.d0-1,0,6);//only arm path now
  fil.close();
  q_index = 0;
  q_ind2 = 0;
  MT::getParameter(mult,"mult");
  //mult = 4; //8;//9
}


int main(int argc,char** argv){
  MT::IOraw = true;
  MT::initCmdLine(argc,argv);
  signal(SIGINT,RobotProcessGroup::signalStopCallback);
  RobotProcessGroup robotProcesses;
  MyDemo demo;
  robotProcesses.ctrl.task=&demo;
  robotProcesses.open();
  robotProcesses.ctrl.ors.getBodyByName("obstacle")->X.pos(0) = 100;
  robotProcesses.gui.ors->getBodyByName("obstacle")->X.pos(0) = 100;
  robotProcesses.gui.ors2->getBodyByName("obstacle")->X.pos(0) = 100;
  demo.loadPlainTrajectory("Q.txt");
  demo.init(&robotProcesses);
  cout << "switch evis display off to get correct timings !!!!!!!!" << endl << endl;
  demo.followTrajectory(&robotProcesses);
  robotProcesses.close();
  return 0;
}


