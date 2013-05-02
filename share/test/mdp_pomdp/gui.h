#undef Unsorted
#undef Bool
#undef None
//#include <Qt/qmetatype.h>
//#include <Qt/qobject.h>
#include <Qt/qfiledialog.h>
#include "gui_ui.h"
#define MT_IMPLEMENT_TEMPLATES
#include <MT/util.h>
#include <MT/mdp_EMSolver.h>

class Gui:public QDialog{
Q_OBJECT
public:
  Ui_PomdpDialog ui;
  mdp::EMSolver sol;
  bool isRunning;
  Gui(){
    ui.setupUi(this);
    connect(ui.go, SIGNAL(clicked()), this, SLOT(toggle_loop()));
    connect(ui.resetFsc, SIGNAL(clicked()), this, SLOT(initFsc()));
    connect(ui.gnuplotByTime, SIGNAL(clicked()), this, SLOT(gnuplotByTime()));
    connect(ui.gnuplotByIter, SIGNAL(clicked()), this, SLOT(gnuplotByIter()));
    connect(ui.reportParams, SIGNAL(clicked()), this, SLOT(reportParameters()));
    connect(ui.problemBrowse, SIGNAL(clicked()), this, SLOT(problemBrowse()));
    connect(ui.resetAll, SIGNAL(clicked()), this, SLOT(initProblem()));
    connect(this , SIGNAL(finished(int)), this, SLOT(quit()));
    show();
    isRunning=false;
    ui.readme->setSource(QUrl("README.html#gui"));
  }
  
public slots:
  void reportParameters(){ getParams();  sol.reportParameters(cout); }
  void initProblem(){
    getParams();
    sol.clear();
    sol.initProblem();
    sol.initFsc();
    cout <<"\n*** MDP:" <<endl;    mdp::writeMDP_fg(sol.mdps,cout,true);
    cout <<"\nfull normalization of MDP = " << checkNormalization(sol.mdps) <<" (should be cardinality of action variables)" <<endl;
    cout <<"\n*** FSC:" <<endl;    mdp::writeFSC_fg(sol.fsc,cout,true);
  }
  void initFsc(){ sol.seed++;  ui.seed->setValue(sol.seed);  getParams();  sol.initFsc();  }
  void gnuplotByTime(){ sol.gnuplot(true); }
  void gnuplotByIter(){ sol.gnuplot(false); }
  
  void problemBrowse(){
    QString file = QFileDialog::getOpenFileName(this,
        QString("POMDP problem file selection"),
        ui.problem->text(),
        QString("*.mdp_arr *.mdp_fg *.ddgm.tabular *.POMDP\n*"));//,QString(" choose a POMDP file") );
    ui.problem->setText(file);
    initProblem();
  }
  
  void getParams(){
    sol.problemFile  = ui.problem->text().toAscii();
    sol.outputPrefix = ui.prefix->text().toAscii();
    sol.fscFile      = ui.fscFile->text().toAscii();
    sol.seed = ui.seed->value();
    sol.fscType = (mdp::FscType)ui.fscType->currentIndex();
    sol.levels.setText(ui.levels->text().toAscii());
    sol.mstepType = (mdp::MstepType)ui.mstepType->currentIndex();
    sol.mstepRate = ui.mstepRate->value();
    sol.mstepNoise = ui.mstepNoise->value();
    sol.estepIncremental = ui.estepIncremental->isChecked();
    sol.estepStructured  = ui.estepStructured->isChecked();
    sol.estepHorizon     = ui.estepHorizon->value();
  }
  
  void go(){
    getParams();
    if(!sol.mdps.facs.N){ initProblem(); }
    if(!sol.fsc.vars.N){ sol.initFsc(); }
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
    }else{
      ui.go->setText("PAUSE");
      isRunning=true;
      go();
    }
  }

  void quit(){
    gnuplotClose();
    cout <<"bye bye" <<endl;
    exit(0);
  }
};
