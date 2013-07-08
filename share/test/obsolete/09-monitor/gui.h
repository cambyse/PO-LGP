#define QT3_SUPPORT
#include <Qt/qobject.h>
#include <Qt/qfiledialog.h>
#include <Qt/qtimer.h>
#include "gui_ui.h"
#include <Core/array.h>
#include <MT/schunk.h>
#include <Core/util.h>
#include <lwa/Device/Device.h>

class Gui:public QDialog{
Q_OBJECT
public:
  Ui_SchunkMonitor ui;
  QTimer pulse;

  floatA q_desired,q_off;
  floatA q_real,v_real,c_real;
  SchunkArmModule schunk;

  Gui(){
    ui.setupUi(this);
    show();

    q_desired.resize(7);
    q_real.resize(7);
    v_real.resize(7);
    c_real.resize(7);

    schunk.open();
    update();
    schunk.reportParameters(cout);
    q_desired = q_real;

    ui.t3->setValue(500.*q_desired(0));
    ui.t4->setValue(500.*q_desired(1));
    ui.t5->setValue(500.*q_desired(2));
    ui.t6->setValue(500.*q_desired(3));
    ui.t7->setValue(500.*q_desired(4));
    ui.t8->setValue(500.*q_desired(5));
    ui.t9->setValue(500.*q_desired(6));

    connect(ui.t3, SIGNAL(valueChanged(int)), this, SLOT(changeTarget3(int)));
    connect(ui.t4, SIGNAL(valueChanged(int)), this, SLOT(changeTarget4(int)));
    connect(ui.t5, SIGNAL(valueChanged(int)), this, SLOT(changeTarget5(int)));
    connect(ui.t6, SIGNAL(valueChanged(int)), this, SLOT(changeTarget6(int)));
    connect(ui.t7, SIGNAL(valueChanged(int)), this, SLOT(changeTarget7(int)));
    connect(ui.t8, SIGNAL(valueChanged(int)), this, SLOT(changeTarget8(int)));
    connect(ui.t9, SIGNAL(valueChanged(int)), this, SLOT(changeTarget9(int)));

    connect(ui.homeOffset, SIGNAL(clicked()), this, SLOT(setThisPosAsHomeOffset()));
    connect(ui.reportAll , SIGNAL(clicked()), this, SLOT(report()));

    connect(&pulse, SIGNAL(timeout()), this, SLOT(update()));
    pulse.start(100);
  }
  
public slots:
  void update(){
    //cout <<"update..." <<endl;
    schunk.getState(q_real,v_real,c_real);
    updateDisp();
  }

  void changeTarget3(int val){ changeTarget(3,val); }
  void changeTarget4(int val){ changeTarget(4,val); }
  void changeTarget5(int val){ changeTarget(5,val); }
  void changeTarget6(int val){ changeTarget(6,val); }
  void changeTarget7(int val){ changeTarget(7,val); }
  void changeTarget8(int val){ changeTarget(8,val); }
  void changeTarget9(int val){ changeTarget(9,val); }

  void changeTarget(uint m,int val){
    //cout <<"target change: " <<m <<"=" <<val <<endl;
    q_desired(m-3) = .002 * val;
    schunk.pDev->moveRamp(m, q_desired(m-3), .1, .1); //.3, 3.);
  }

  void updateDisp(){
    ui.p3->setValue(500.*q_real(0));
    ui.p4->setValue(500.*q_real(1));
    ui.p5->setValue(500.*q_real(2));
    ui.p6->setValue(500.*q_real(3));
    ui.p7->setValue(500.*q_real(4));
    ui.p8->setValue(500.*q_real(5));
    ui.p9->setValue(500.*q_real(6));
    
    ui.m3->setText(STRING(std::setprecision(3)<<q_real(0)));
    ui.m4->setText(STRING(std::setprecision(3)<<q_real(1)));
    ui.m5->setText(STRING(std::setprecision(3)<<q_real(2)));
    ui.m6->setText(STRING(std::setprecision(3)<<q_real(3)));
    ui.m7->setText(STRING(std::setprecision(3)<<q_real(4)));
    ui.m8->setText(STRING(std::setprecision(3)<<q_real(5)));
    ui.m9->setText(STRING(std::setprecision(3)<<q_real(6)));

    ui.v3->setText(STRING(std::setprecision(3)<<v_real(0)));
    ui.v4->setText(STRING(std::setprecision(3)<<v_real(1)));
    ui.v5->setText(STRING(std::setprecision(3)<<v_real(2)));
    ui.v6->setText(STRING(std::setprecision(3)<<v_real(3)));
    ui.v7->setText(STRING(std::setprecision(3)<<v_real(4)));
    ui.v8->setText(STRING(std::setprecision(3)<<v_real(5)));
    ui.v9->setText(STRING(std::setprecision(3)<<v_real(6)));
    
  }

  void setThisPosAsHomeOffset(){
    schunk.getOff(q_off);
    cout <<"q_off before = " <<q_off <<endl;

    for(uint m=3;m<=9;m++) schunk.pDev->setHomeOffset(m,0);
    schunk.getState(q_real,v_real,c_real);
    for(uint m=3;m<=9;m++) schunk.pDev->setHomeOffset(m,q_real(m-3));
    update();
    q_desired = q_real;
    
    schunk.getOff(q_off);
    cout <<"q_off before = " <<q_off <<endl;
  }

  void report(){
    schunk.reportParameters(cout);
  }

  /*
    void gnuplot(){
    MT::String cmd;
    cmd <<"plot '" <<sol.outfilename <<"' us 1:3";
    cout <<"gnuplot command: " <<cmd <<endl;
    ::gnuplot(cmd.p());
  }
  
  void go(){
    getParams();
    if(!sol.mdp.Px.N){ sol.init(); sol.initFsc(); }
    if(!sol.fsc.vars.N){ sol.initFsc(); }
    //set button toggle
    sol.resetTimer();
    for(;isRunning;){
      sol.step();
      QCoreApplication::processEvents();
    }
  }
  
  void toggle_loop(){
    if(isRunning){
      ui.go->setText("GO");
      isRunning=false;
      //QCoreApplication::exit_loop();
    }else{
      ui.go->setText("PAUSE");
      isRunning=true;
      go();
      //QCoreApplication::enter_loop();
    }
  }

  void quit(){
    gnuplotClose();
    cout <<"bye bye" <<endl;
    exit(0);
  }
  */
};
