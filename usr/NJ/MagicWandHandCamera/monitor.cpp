#include "monitor.h"

char outputbuf[200];
#define FORMAT(x) (sprintf(outputbuf,"%.2f",x)>0?outputbuf:NULL)
#define FORMATPM(x) (sprintf(outputbuf,"%.2f+-%.2f",x,sqrt(x##Var-x*x))>0?outputbuf:NULL)
#define VELC

             
void glDrawControlLoop(void *P){
  //Monitor *cg=(Monitor*)P;
  //glColor(1.,0.,0.);
  //glDrawDiamond(cl->CV_x->x(0),cl->CV_x->x(1),cl->CV_x->x(2),.02,.02,.02);
}

Monitor::Monitor(){
  logData=plotData=false;
  isOpen=false;
  useOpengl=true;
}

void Monitor::open(){
#ifdef MT_QT
  app = new QApplication(MT::argc,MT::argv);
  app->processEvents();
  
  ui = &(new Gui())->ui; //;Ui_SchunkMonitor();
#endif
  
  //-- ors
 //-- ors
  if(MT::checkParameter<MT::String>("orsFile")){
	String sfile;
        MT::getParameter<MT::String>(sfile,"orsFile");
	ors <<FILE(sfile);
}
else
  ors <<FILE("../../../share/configurations/schunk.ors");

  ors.calcNodeFramesFromEdges();

#ifdef MT_QT
  gl = new OpenGL(ui->glparent,"hallo",401,401,0,0);
#else
  gl = new OpenGL();
#endif
    
  //-- opengl
  gl->add(glStandardScene,0);
  //gl->add(glDrawControlLoop,this);
  gl->add(ors::glDrawGraph,&ors);
  gl->camera.setPosition(7.,-0.,2.);
  gl->camera.focus(0,0,.8);
  gl->update();
  isOpen=true;
}

void Monitor::step(){
  try{
    ors.setJointState(q_gui);
    ors.calcNodeFramesFromEdges();
    if(useOpengl) gl->update();
    
    /*if(plotData && !(loopCounter%20)){
    write(history_time,history_q_target,history_q_real,history_v_real,history_c_real,"z.plotData");
    gnuplot("plot 'z.plotData' us 1:2 notitle ,\
    'z.plotData' us 1:3 notitle ,\
    'z.plotData' us 1:4 notitle ,\
    'z.plotData' us 1:5 notitle ,\
    'z.plotData' us 1:6 notitle ,\
    'z.plotData' us 1:7 notitle ,\
    'z.plotData' us 1:8 notitle");
  }*/

#ifdef MT_QT
    if(ctrl->loopCounter){
      ui->time->setText(FORMAT(ctrl->lastTime));
      ui->cycleTime->setText(FORMATPM(ctrl->cycleTime));
      ui->cycleBusyTime->setText(FORMATPM(ctrl->cycleBusyTime));
      ui->cycleTimeControl->setText(FORMAT(1000.*ctrl->cycleTimeControl));
      ui->cycleTimeArm ->setText(FORMATPM(ctrl->arm .threadStepTime));
      ui->cycleTimeHand->setText(FORMATPM(ctrl->hand.threadStepTime));
      ui->cycleTimeSkin->setText(FORMATPM(ctrl->skin.threadStepTime));
      ui->cycleTimeMonitor->setText(FORMATPM(ctrl->gui->threadStepTime));
      ui->collisionCost->setText(FORMAT(ctrl->CV_col->x(0)));
      ui->limitCost->setText(FORMAT(ctrl->CV_lim->x(0)));
      ui->joyState->setText(STRING(ctrl->joyState));
    }
    
    /*if(arm.readPositions){
    ui->p3->setValue(500.*ctrl->q_real(0));
    ui->p4->setValue(500.*ctrl->q_real(1));
    ui->p5->setValue(500.*ctrl->q_real(2));
    ui->p6->setValue(500.*ctrl->q_real(3));
    ui->p7->setValue(500.*ctrl->q_real(4));
    ui->p8->setValue(500.*ctrl->q_real(5));
    ui->p9->setValue(500.*ctrl->q_real(6));
    
    ui->m3->setText(FORMAT(ctrl->q_real(0)));
    ui->m4->setText(FORMAT(ctrl->q_real(1)));
    ui->m5->setText(FORMAT(ctrl->q_real(2)));
    ui->m6->setText(FORMAT(ctrl->q_real(3)));
    ui->m7->setText(FORMAT(ctrl->q_real(4)));
    ui->m8->setText(FORMAT(ctrl->q_real(5)));
    ui->m9->setText(FORMAT(ctrl->q_real(6)));
  }*/

  /*if(readAll){
    ui->v3->setText(FORMAT(v_real(0)));
    ui->v4->setText(FORMAT(v_real(1)));
    ui->v5->setText(FORMAT(v_real(2)));
    ui->v6->setText(FORMAT(v_real(3)));
    ui->v7->setText(FORMAT(v_real(4)));
    ui->v8->setText(FORMAT(v_real(5)));
    ui->v9->setText(FORMAT(v_real(6)));
  }*/

    app->processEvents();
    
  //readPositions = ui->readPositions->isChecked();
    logData = ui->logData->isChecked();
    plotData = ui->plotData->isChecked();
    useOpengl = ui->opengl->isChecked();

    /*q_gui(0) = .002 * ui->t3->value();
    q_gui(1) = .002 * ui->t4->value();
    q_gui(2) = .002 * ui->t5->value();
    q_gui(3) = .002 * ui->t6->value();
    q_gui(4) = .002 * ui->t7->value();
    q_gui(5) = .002 * ui->t8->value();
    q_gui(6) = .002 * ui->t9->value();
    */
#endif
  }
  catch(const char *msg){
    MT_MSG("exception caught: "<<msg);
  }
}

void Monitor::close(){
  delete gl;
#ifdef MT_QT
  delete app;
  //TOTO!!ui = &(new Gui())->ui; //;Ui_SchunkMonitor();
#endif
}

