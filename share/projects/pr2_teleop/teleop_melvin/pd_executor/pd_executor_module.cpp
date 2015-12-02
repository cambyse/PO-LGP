#include "pd_executor_module.h"
#include <Ors/ors.h>
#include <Motion/pr2_heuristics.h>


//#ifdef WITH_ROS
  //  #include <pr2/roscom.h>
//#endif

// ############################################################################
// Executor
PDExecutor::PDExecutor()
    : Module("PDExecutor", .01), world("model.kvg"),worldreal("model.kvg"), fmc(world, true), tele2tasks(fmc), useros(false)
{

  // fmc setup
  world.getJointState(q, qdot);
  fmc.H_rate_diag = pr2_reasonable_W(world);
  fmc.qitselfPD.y_ref = q;
  fmc.qitselfPD.setGains(2.3,10);
//  fmc.qitselfPD.active = false;
  fmc.qitselfPD.prec = 0.100;
  //fmc.qitselfPD.maxVel = 0.008;
  //fmc.qitselfPD.maxAcc = 0.09;
  //fmc.qitseldPD.f_Igain = 1.;
}


void setOdom(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose){

  ors::Quaternion quat(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
  ors::Vector pos(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);

  double angle;
  ors::Vector rotvec;
  quat.getRad(angle, rotvec);
  q(qIndex+0) = pos(0);
  q(qIndex+1) = pos(1);
  q(qIndex+2) = mlr::sign(rotvec(2)) * angle;
//  cout<<q<<endl;

}



void PDExecutor::step()
{

    //cout<<"\x1B[2J\x1B[H";
    if (useros && !inited)
    {
        cout << "STARTING TO OPEN" << endl;
        initRos();
        cout << "FINISHED TO OPEN" << endl;
        inited = true;
    }

    world.watch(false);
    worldreal.watch(false);
    
    ors::Joint *trans= world.getJointByName("worldTranslationRotation");





    //if(length(cal_pose_lh)==0||length(cal_pose_rh)==0) return;
    bool init;
    init = initmapper.get();


    if(!init)
    {
      tele2tasks.updateTasks(calibrated_pose_rh.get(), calibrated_pose_lh.get(), calibrated_gripper_lh.get(), calibrated_gripper_rh.get(), drive.get());
    }else{
      tele2tasks.deactivateTasks();
    }

    double tau = 0.001;

    for (uint t = 0; t < 20 ; t++)    {
        arr a = fmc.operationalSpaceControl();
        q += tau * qdot;
        qdot += tau * a;
               
        fmc.setState(q, qdot);
  
    }

    // set state
    CtrlMsg ref;
    ref.q = q;
    arr qdotzero;
    qdotzero.resizeAs(q).setZero();
    ref.qdot = qdotzero;

    ref.u_bias = zeros(q.N);

    ref.fL =zeros(6);
    ref.fR =zeros(6);
    ref.Kp = {1.};
    ref.Ki.clear();
    ref.Kd = {1.};
    ref.gamma = .988;
    ref.J_ft_inv.clear();
    //fmc.reportCurrentState();
    if ( trans && trans->qDim()==3)
    {
        ref.qdot(trans->qIndex+0) = qdot(trans->qIndex+0);
        ref.qdot(trans->qIndex+1) = qdot(trans->qIndex+1);
        ref.qdot(trans->qIndex+2) = qdot(trans->qIndex+2);
   }

    /*  FORCE CONTROL
    if(useros)
    {
        uint count=0;
        //ctrlTasks.readAccess();
        //fm.tasks = ctrlTasks();
        if(fLobs.N && uobs.N)
        {

            for(CtrlTask *t : fmc.tasks)
            {
         
                if(t->active && t->f_ref.N)
                {
                    count++;
                    if(count!=1) HALT("you have multiple active force control tasks - NIY");
                    t->getForceControlCoeffs(ref.fL, ref.u_bias, ref.Ki, ref.J_ft_inv, worldreal);
                }
            }     
        }
        if(count==1 )
        {
            for(uint i = 0;i<fLobs.N;i++)
            {
                  if(fLobs(i) > fc->f_ref(i))  error(i) =ref.gamma*error(i)+ fc->f_ref(i)-fLobs(i);
                  else if(fLobs(i) < -fc->f_ref(i)) error(i) =ref.gamma*error(i)+ -fc->f_ref(i)-fLobs(i);
                  else error(i)=0.;
            }
        }    
        //  cout<<"fl"<<ref.fL<<endl<<"ubias"<<ref.u_bias<<endl<<"KI"<<ref.Ki<<endl<<"J_ft"<< ref.J_ft_inv<<endl;
        // ctrlTasks.deAccess();
    
        //-- send the computed movement to the robot
       // cout<<"error"<<error<<endl;
       // ctrl_ref.set() = ref;
    }
  */
  ctrl_ref.set() = ref;

}

void PDExecutor::sendRosCtrlMsg()
{

  CtrlMsg ref;
  ref.q = q;
  arr qdotzero;
  qdotzero.resizeAs(q).setZero();
  ref.qdot = qdotzero;

  ref.u_bias = zeros(q.N);

  ref.fL =zeros(6);
  ref.fR =zeros(6);
  ref.Kp = {1.};
  ref.Ki.clear();
  ref.Kd = {1.};
  ref.gamma = .5;
  ref.J_ft_inv.clear();
  ctrl_ref.set() = ref;
 //  roscom->publishJointReference();
//#endif

}

void PDExecutor::initRos()
{

//#ifdef WITH_ROS
  // if (roscom == nullptr)
  //   return;

  cout << "** Waiting for ROS message on initial configuration.." << endl;
  // get robot state from the robot
  for (;;) {
    ctrl_obs.var->waitForNextRevision();
    CtrlMsg obs = ctrl_obs.get();

    cout << "================================================\n"
         << "  observed q.N:    " << obs.q.N << "\n"
         << "  world q.N:       " << world.q.N  << "\n"
         << "  observed qdot.N: " << obs.qdot.N << "\n"
         << "  world qdot.N:    " << world.qdot.N << "\n"
         << "================================================" << endl;

    if (obs.q.N == world.q.N && obs.qdot.N == world.qdot.N)
      break;

  }

  cout << "** Setting State of robot in simulation" << endl;
  // set robot state of the fmc
  q = ctrl_obs.get()->q;
  qdot = ctrl_obs.get()->qdot;
  fmc.setState(q, qdot);
  worldreal.setJointState(q,qdot);
  cout << "DONE" << endl;
//#endif


}


void PDExecutor::open()
{

  useros = mlr::getParameter<bool>("useRos", false);
  error = zeros(3);

}

void PDExecutor::close()
{

    CtrlMsg ref;
    ref.q = q;
    arr qdotzero;
    qdotzero.resizeAs(q).setZero();
    ref.qdot = qdotzero;

    ref.u_bias = zeros(q.N);

    ref.fL =zeros(6);
    ref.fR =zeros(6);
    ref.Kp = {1.};
    ref.Ki.clear();
    ref.Kd = {1.};
    ref.gamma = 0.;
    ref.J_ft_inv.clear();
    //ctrl_ref.set() = ref;

}
