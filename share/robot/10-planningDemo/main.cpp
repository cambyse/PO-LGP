#include <MT/robot.h>
#include <MT/perceptionModule.h>
#include <MT/motionPlannerModule.h>
#include <MT/robot_marcTask.h>
#include <MT/robotActionInterface.h>
#include <TL/decisionMakingModule.h>
#include <signal.h>

int main(int argn,char** argv) {
    MT::initCmdLine(argn,argv);
    signal(SIGINT,RobotModuleGroup::signalStopCallback);

    RobotActionInterface R;
    R.open();

    //********** launch all modules
    RobotModuleGroup &master = *R.getProcessGroup();
    PerceptionModule perc;
    perc.input=&master.evis.output;
    //MotionPlannerModuleGroup motion;
    //DecisionMakingModule brain;
    master.gui.perceptionOutputVar=&perc.output;

    TaskAbstraction& task = *R.getTask();
    //master.ctrl.task=&task;

    //robot group
    //master.open();

    //perc
    perc.threadOpen();

    //motion
    FutureMotionPlan planVar;
    FutureMotionGoal goalVar;
    ReceedingHorizonProcess planner;
    planner.planVar = &planVar;
    planner.goalVar = &goalVar;
    planner.sys_parent = &master.ctrl.sys;

    master.gui.planVar  = &planVar;
    task.planVar = &planVar;
    task.joyVar  = &master.joy;

    planner.threadOpen();

    //brain
    //brain.ors = master.ctrl.sys.ors;
    //brain.threadOpen();

    task.controlMode=stopCM;
    int STATE=-1;
    if (!master.openBumble) STATE=0;

    //********** main loop
    for (;!master.signalStop && master.joy.state(0)!=16 && master.joy.state(0)!=32;) {

        //brain -> motion
        /*if(brain.actionIsReady && brain.action!=DecisionMakingModule::SYMBOLIC_ACTION__NO_ACTION_FOUND){
          motion.graspTargetBodyId = brain.actionArgument;
        }*/
        goalVar.writeAccess(NULL);
        planVar.readAccess(NULL);

        switch (STATE) {
        case -1: { //wait for perception to find at least 3 objects
            perc.output.readAccess(NULL);
            if (perc.output.objects.N>=3 && perc.output.objects(0).found>3 && perc.output.objects(1).found>3 && perc.output.objects(2).found>3) {
                ors::Shape *s=master.ctrl.ors.getShapeByName("cyl1");
                s->rel.pos.set(perc.output.objects(0).center3d.p);
                s->rel.pos -= s->body->X.pos;
                s=master.ctrl.ors.getShapeByName("cyl2");
                s->rel.pos.set(perc.output.objects(1).center3d.p);
                s->rel.pos -= s->body->X.pos;
                master.gui.ors->copyShapesAndJoints(master.ctrl.ors);
                master.gui.ors2->copyShapesAndJoints(master.ctrl.ors);
                STATE ++;
            }
#if 0 //hard-set an object independent of perception - for debugging
            static uint count=0;
            count++;
            if (count==10) {
                ors::Shape *s=master.ctrl.ors.getShapeByName("cyl1");
                s->rel.pos.set(ARR(0,-1,1).p);
                s->rel.pos -= s->body->X.pos;
                master.gui.ors->copyShapesAndJoints(master.ctrl.ors);
                master.gui.ors2->copyShapesAndJoints(master.ctrl.ors);
                STATE ++;
            }
#endif
            perc.output.deAccess(NULL);
        } break;
        case 0: { //grasp plan and execute
            goalVar.goalType=graspGoalT;
            goalVar.graspShape="cyl1";
            if (planVar.converged) {
                task.controlMode=followTrajCM;
            }
            if (planVar.executed) {
                task.controlMode=stopCM;
                goalVar.goalType=noGoalT;
                STATE++;
            }
        } break;
        case 1: { //stop and reattach object
            task.controlMode=stopCM;
            static int count=0;
            count++;
            if (count>50) {
                reattachShape(master.ctrl.ors, &master.ctrl.swift, "cyl1", "m9", "table");
                reattachShape(*master.gui.ors, NULL, "cyl1", "m9", NULL);
                reattachShape(*master.gui.ors2, NULL, "cyl1", "m9", NULL);
                STATE++;
            }
        } break;
        case 2: { //close hand
            task.controlMode=closeHandCM;
            master.ctrl.forceColLimTVs=false;
            static int count=0;
            count++;
            if (count>300) {
                master.ctrl.forceColLimTVs=true;
                task.controlMode=stopCM;
                STATE++;
            }
        } break;
        case 3: { //wait until planner is ready with current step, then reset the planner
            task.controlMode=stopCM;
            if (planner.threadIsIdle()) {
                planVar.converged=false;
                planVar.executed=false;
                planVar.ctrlTime=0.;
                STATE++;
            }
        } break;
        case 4: { //place plan and execute
            goalVar.goalType=placeGoalT;
            goalVar.graspShape="cyl1";
            goalVar.belowFromShape="table";
            goalVar.belowToShape="cyl2";
            if (planVar.converged) {
                master.ctrl.fixFingers=true;
                task.controlMode=followTrajCM;
            }
            if (planVar.executed) {
                task.controlMode=stopCM;
                master.ctrl.fixFingers=false;
                goalVar.goalType=noGoalT;
                STATE++;
            }
        } break;
        case 5: { //stop motion
            task.controlMode=stopCM;
            static int count=0;
            count++;
            if (count>50) {
                STATE++;
            }
        } break;
        case 6: { //openHand
            task.controlMode=openHandCM;
            master.ctrl.forceColLimTVs=false;
            static int count=0;
            count++;
            if (count>300) {
                master.ctrl.forceColLimTVs=true;
                task.controlMode=stopCM;
                reattachShape(master.ctrl.ors, &master.ctrl.swift, "cyl1", "OBJECTS", "cyl2");
                reattachShape(*master.gui.ors, NULL, "cyl1", "OBJECTS", NULL);
                reattachShape(*master.gui.ors2, NULL, "cyl1", "OBJECTS", NULL);
                STATE++;
            }
        } break;
        case 7: { //wait until planner is ready with current step
            task.controlMode=stopCM;
            if (planner.threadIsIdle()) {
                planVar.converged=false;
                planVar.executed=false;
                planVar.ctrlTime=0.;
                STATE++;
            }
        } break;
        case 8: { //plan a homing trajectory and execute
            goalVar.goalType=homingGoalT;
            goalVar.graspShape="cyl1"; //do we need this?
            goalVar.belowToShape="cyl2"; //do we need this?
            if (planVar.converged) {
                task.controlMode=followTrajCM;
            }
            if (planVar.executed) {
                task.controlMode=stopCM;
                goalVar.goalType=noGoalT;
                STATE++;
            }
        } break;
        case 9: { //wait until planner is ready; reset; and start from -1 again...
            task.controlMode=stopCM;
            static int count=0;
            count++;
            if (planner.threadIsIdle() && count>500) {
                planVar.converged=false;
                planVar.executed=false;
                planVar.ctrlTime=0.;
                if (perc.output.objects.N>=3) {
                    perc.output.objects(0).found=perc.output.objects(1).found=perc.output.objects(2).found=0;
                }
                STATE=-1;
                if (!master.openBumble) STATE=0;
                STATE=20;
            }
        }
        case 20: {
            task.controlMode=stopCM;
        } break;
        case 21: {
            task.controlMode=homingCM;
        } break;
        default:
            HALT("");
        }


        cout <<'\r';
        planVar.write(cout);
        cout <<" controlMode= " <<task.controlMode <<flush;

        goalVar.deAccess(NULL);
        planVar.deAccess(NULL);

        //------------STEPPING
        master.step();
        planner.threadStepOrSkip(0);
        perc.threadStepOrSkip(0);
        //brain.threadStepOrSkip(0);

        //reportGlobalProcessGraph();
    }


    //brain.threadClose();
    planner.threadClose();
    perc.close();
    master.close();
}
