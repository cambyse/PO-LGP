#include "guiModule.h"
#include "ors.h"
#include "perceptionModule.h"
#include "plot.h"
#include "opengl.h"
#include <NP/camera.h>

GuiModule::GuiModule():Process("GuiProcess"){
  q_referenceVar=NULL;
  proxiesVar=NULL;
  cameraVar=NULL;
  perceptionOutputVar=NULL;
  planVar=NULL;
  logData=plotData=false;
  ors=ors2=NULL;
}

void GuiModule::createOrsClones(ors::Graph *_ors){
  if(_ors) ors  = _ors->newClone(); else ors =NULL;
  if(_ors) ors2 = _ors->newClone(); else ors2=NULL;
}

void GuiModule::open(){
// #ifdef MT_QT
//   app = new QApplication(MT::argc, MT::argv);
//   app->processEvents();
  
//   ui = &(new Gui())->ui; //;Ui_SchunkMonitor();
// #endif
  
  /*//-- ors
  MT::load(ors, "../../../share/configurations/schunk.ors", true);
  ors.calcNodeFramesFromEdges();
  */
  
#ifdef MT_QT
  //gl = new OpenGL(ui->glparent, "hallo", 401, 401, 0, 0);
#else
  gl = new OpenGL("GuiModule", 2*512, 2*384);
#endif
  
  //-- opengl
  gl->setViewPort(0, .0, .5, .5, 1.);
  gl->setViewPort(1, .0, .5, .0, .5);
  gl->setViewPort(2, .5, 1., .5, 1.);
  gl->setViewPort(3, .5, 1., .0, .5);
  
  if(ors){
    gl->addView(0, glStandardLight, NULL);
    gl->addView(0, ors::glDrawGraph, ors);
    gl->views(0).camera.setPosition(7., -0., 2.);
    gl->views(0).camera.focus(0, 0, .8);
    gl->views(0).camera.upright();
    
    gl->addView(2, glStandardLight, NULL);
    gl->addView(2, ors::glDrawGraph, ors2);
    gl->views(2).camera.setPosition(7., -0., 2.);
    gl->views(2).camera.focus(0, 0, .8);
    gl->views(2).camera.upright();
    
    orsDrawJoints=false;
  }
  
  gl->addView(0, glDrawPlot, &plotModule);
  
  gl->update();
  //dispSteps = 0;
}

void GuiModule::step(){
  MT::Array<Object> objects;
  arr q_reference;  // joint state of current robot
  arr q_trajectory; // a trajectory to display
  
  processLock.writeLock();
  
  if(q_referenceVar){
    q_referenceVar->readAccess(this);
    q_reference = q_referenceVar->q_reference;
    q_referenceVar->deAccess(this);
  }//else MT_MSG("Variable pointer q_referenceVar not set");
  
  if(proxiesVar){
    proxiesVar->readAccess(this);
    listCopy(ors->proxies, proxiesVar->proxies);
    proxiesVar->deAccess(this);
  }//else MT_MSG("Variable pointer proxiesVar not set");
  
  if(perceptionOutputVar && ors){
    perceptionOutputVar->readAccess(this);
    img[3] = perceptionOutputVar->disp;
    objects = perceptionOutputVar->objects;
    perceptionOutputVar->deAccess(this);
    realizeObjectsInOrs(*ors, objects);
    copyBodyInfos(*ors2, *ors);
  }//else  MT_MSG("Variable pointer perceptionOutputVar or ors not set");
  if(planVar){
    planVar->readAccess(this);
    q_trajectory = planVar->q;
    planVar->deAccess(this);
  }
  if(cameraVar){
    cameraVar->readAccess(this);
    img[1] = cameraVar->rgbL;
    cameraVar->deAccess(this);
  }// Nikolay change: display spammed else MT_MSG("Variable pointer cameraVar not set");
  
  //if(img[0].N) gl->img=&img[0];
  for(uint i=0; i<6; i++) if(img[i].N) gl->views(i).img=&img[i];
  
  if(objects.N && img[1].N){
    Object *obj;
    for_elem(obj, objects){//draw 2d percepetion shapes
      if(!obj->found) continue;
      cvDrawPoints(img[1], obj->shapePointsL);
      cvDrawPoints(img[1], obj->shapePointsR);
    }
    
    plotClear();
    for_elem(obj, objects){//draw estimated shapes from perception as simple points
      if(!obj->found) continue;
      plotPoints(obj->shapePoints3d);
      plotPoints(obj->center3d);
    }
  }
  
  if(linesToDisplay.N){
    plotClear();
    for(uint i=0; i<linesToDisplay.N; i++) plotLine(linesToDisplay(i));
  }
  
  //display the current state
  if(ors && q_reference.N){
    ors->setJointState(q_reference);
    ors->calcBodyFramesFromJoints();
  }
  
  //display a certain step of the trajectory
  if(ors2 && q_trajectory.N){
    //static uint k=0;
    uint T=q_trajectory.d0-1, t; //num;
    //if(dispSteps==1 || dispSteps<=0) num=T; else num=dispSteps;
    //t=k*T/num;
    //k++;  if(k>=num) k=0;
    t=T;
    ors2->setJointState(q_trajectory[t]);
    //if(q_external.N)  ors2->setExternalState(q_external[t]);
    ors2->calcBodyFramesFromJoints();
    gl->text.clear() <<"t=" <<t;
  }
  
  gl->update();
  // gl->watch();
  
  processLock.unlock();
  
  /*if(plotData && !(loopCounter%20)){
    write(history_time, history_q_target, history_q_real, history_v_real, history_c_real, "z.plotData");
    gnuplot("plot 'z.plotData' us 1:2 notitle , \
    'z.plotData' us 1:3 notitle , \
    'z.plotData' us 1:4 notitle , \
    'z.plotData' us 1:5 notitle , \
    'z.plotData' us 1:6 notitle , \
    'z.plotData' us 1:7 notitle , \
    'z.plotData' us 1:8 notitle");
  }*/
  
#ifdef MT_QT
//   if(ctrl->stepCounter){
//     ui->time            ->setText(FORMAT(ctrl->lastTime));
//     ui->cycleTime       ->setText(FORMATPM(ctrl->cycleTime));
//     ui->cycleBusyTime   ->setText(FORMATPM(ctrl->cycleBusyTime));
//     ui->cycleTimeControl->setText(FORMATPM(ctrl->cycleCtrlTime));
//     ui->cycleTimeArm    ->setText(FORMATPM(ctrl->arm .threadStepTime));
//     ui->cycleTimeHand   ->setText(FORMATPM(ctrl->hand.threadStepTime));
//     ui->cycleTimeSkin   ->setText(FORMATPM(ctrl->skin.threadStepTime));
//     ui->cycleTimeMonitor->setText(FORMATPM(ctrl->gui->threadStepTime));
// //       ui->collisionCost   ->setText(FORMAT(ctrl->CV_col->x(0)));
// //       ui->limitCost       ->setText(FORMAT(ctrl->CV_lim->x(0)));
//     ui->joyState        ->setText(STRING(ctrl->joy.state));
//   }
  
//   /*if(arm.readPositions){
//   ui->p3->setValue(500.*ctrl->q_real(0));
//   ui->p4->setValue(500.*ctrl->q_real(1));
//   ui->p5->setValue(500.*ctrl->q_real(2));
//   ui->p6->setValue(500.*ctrl->q_real(3));
//   ui->p7->setValue(500.*ctrl->q_real(4));
//   ui->p8->setValue(500.*ctrl->q_real(5));
//   ui->p9->setValue(500.*ctrl->q_real(6));
  
//   ui->m3->setText(FORMAT(ctrl->q_real(0)));
//   ui->m4->setText(FORMAT(ctrl->q_real(1)));
//   ui->m5->setText(FORMAT(ctrl->q_real(2)));
//   ui->m6->setText(FORMAT(ctrl->q_real(3)));
//   ui->m7->setText(FORMAT(ctrl->q_real(4)));
//   ui->m8->setText(FORMAT(ctrl->q_real(5)));
//   ui->m9->setText(FORMAT(ctrl->q_real(6)));
//   }*/
  
//   /*if(readAll){
//     ui->v3->setText(FORMAT(v_real(0)));
//     ui->v4->setText(FORMAT(v_real(1)));
//     ui->v5->setText(FORMAT(v_real(2)));
//     ui->v6->setText(FORMAT(v_real(3)));
//     ui->v7->setText(FORMAT(v_real(4)));
//     ui->v8->setText(FORMAT(v_real(5)));
//     ui->v9->setText(FORMAT(v_real(6)));
//   }*/
  
//   app->processEvents();
  
//   //readPositions = ui->readPositions->isChecked();
//   logData = ui->logData->isChecked();
//   plotData = ui->plotData->isChecked();
  
//   /*q_gui(0) = .002 * ui->t3->value();
//   q_gui(1) = .002 * ui->t4->value();
//   q_gui(2) = .002 * ui->t5->value();
//   q_gui(3) = .002 * ui->t6->value();
//   q_gui(4) = .002 * ui->t7->value();
//   q_gui(5) = .002 * ui->t8->value();
//   q_gui(6) = .002 * ui->t9->value();
//   */
#endif
}

void GuiModule::close(){
  delete gl;
  gl = NULL;
#ifdef MT_QT
//   delete app;
//   app = NULL;
//   //TOTO!!ui = &(new Gui())->ui; //;Ui_SchunkMonitor();
#endif
}

GuiModule::~GuiModule(){
  if(gl) delete gl;
}
