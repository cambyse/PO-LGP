#include "act_LGP.h"

#include <LGP/optLGP.h>

struct sAct_LGP : Thread{
  Access<arr> x;
  mlr::KinematicWorld kin;
  FOL_World fol;
  mlr::String fixedSequence;

  OptLGP *lgp;

  Signaler *status;
  OrsPathViewer *viewer=NULL;

  sAct_LGP(Signaler *status)
    : Thread("Act_LGP", 0.), x(this, "PathOpt_x"), lgp(NULL), status(status){}
  ~sAct_LGP(){
      threadClose();
      if(lgp) delete lgp;
  }

  virtual void open();
  virtual void step();
  virtual void close();
};


Act_LGP::Act_LGP(Roopi* r)
  : Act(r), s(NULL) {
  s = new sAct_LGP(this);
}

Act_LGP::~Act_LGP(){
//  if(s->viewer) delete s->viewer;
  delete s;
}

void Act_LGP::setKinematics(const char *kinFile){
    s->kin.init(kinFile);
}

void Act_LGP::setLogic(const char *folFile){
    s->fol.init(folFile);
}

void Act_LGP::fixLogicSequence(const mlr::String& seq){
    s->fixedSequence = seq;
}

mlr::KinematicWorld &Act_LGP::kin(){
    return s->kin;
}

FOL_World &Act_LGP::fol(){
    return s->fol;
}

//OptLGP &Act_LGP::opt(){
//    return *s->lgp;
//}

void Act_LGP::start(){ s->threadLoop(); }

void Act_LGP::stop(){ s->threadStop(); }

void sAct_LGP::open(){
    CHECK(!lgp,"was already opened??");
    CHECK(kin.q.N, "kinematics is not initialized!");
    CHECK(fol.KB.N, "logic is not initialized");
    lgp = new OptLGP(kin, fol);
}

void sAct_LGP::step(){
    if(fixedSequence.N){
        lgp->optFixedSequence(fixedSequence);
//        lgp->renderToFile(3,"z.path.");

        status->setStatus(AS_converged);
        threadStop();
    }else{
        if(!lgp->fringe_expand.N) lgp->init();
        lgp->step();
        uint n = lgp->numFoundSolutions();
        if(n>3){
            //  x.set() = lgp->x;  //TODO: store the solution
            status->setStatus(AS_converged);

            cout <<"OptLGP PathOpt done:\n" <<lgp->report() <<endl;

            threadStop();
        }
    }
}

void sAct_LGP::close(){
    if(lgp) delete lgp; lgp=NULL;
}

