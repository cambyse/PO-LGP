#include <LGP/optLGP.h>

//===========================================================================

struct CooperationDemo{
  mlr::KinematicWorld kin;
  FOL_World fol;
  BodyL box;
  mlr::Frame *tableC;
  mlr::Frame *tableL;
  mlr::Frame *tableR;
  mlr::Array<mlr::Transformation> targetAbs, targetRel;

  CooperationDemo(){
    prepareFol(true);
    prepareKin();
  }

  void prepareKin();
  void prepareFol(bool smaller=false);
};

void CooperationDemo::prepareKin(){
  kin.init("LGP-coop-kin.g");
  computeMeshNormals(kin.shapes);

  tableC = kin.getBodyByName("tableC");
  tableL = kin.getBodyByName("tableL");
  tableR = kin.getBodyByName("tableR");

  {
    //rearrange the assembled box: moving pieces systematically being deassembled
    //grab desired final relative configuration & create initial configuration, placing objects far on the table

    for(mlr::Frame *b:kin.bodies) if(b->name.startsWith("/toolbox")) box.append(b);

    //memorize their relative positionings
    targetAbs.resize(box.N);
    targetRel.resize(box.N, box.N);
    for(uint i=0;i<box.N;i++){
      targetAbs(i) = box(i)->X;
      for(uint j=i+1;j<box.N;j++){
        mlr::Transformation rel;
        rel.setDifference(box(i)->X, box(j)->X);
        targetRel(i,j) = rel;
        if(box(i)->name=="/toolbox/handle" && box(j)->name=="/toolbox/side_front") fol.addValuedFact({"attachable",box(i)->name, box(j)->name}, rel);
        if(box(i)->name=="/toolbox/handle" && box(j)->name=="/toolbox/side_back")  fol.addValuedFact({"attachable",box(i)->name, box(j)->name}, rel);
        if(box(i)->name=="/toolbox/side_front" && box(j)->name=="/toolbox/side_left")  fol.addValuedFact({"attachable",box(i)->name, box(j)->name}, rel);
        if(box(i)->name=="/toolbox/side_front" && box(j)->name=="/toolbox/side_right")  fol.addValuedFact({"attachable",box(i)->name, box(j)->name}, rel);
      }
    }

    //position them on the left table
    double xpos = -.6;
    for(mlr::Frame *b:box){
      mlr::Joint *j = b->inLinks.scalar();
      tableC->outLinks.removeValue(j);
      j->from = tableL;
      tableL->outLinks.append(j);
      kin.checkConsistency();

      j->B.setZero();
      j->B.addRelativeTranslation(xpos, 0,0);
      j->B.addRelativeRotationDeg(90,0,0,1);
      xpos += .15;
    }
    kin.calc_fwdPropagateFrames();
  }
}

void CooperationDemo::prepareFol(bool smaller){
//  fol.verbose = 5;
  fol.init(FILE("LGP-coop-fol.g"));
  //-- prepare logic world
//  for(mlr::Body *b:box) fol.addObject(b->name);
  if(!smaller) fol.addObject("/toolbox/handle");
  if(!smaller) fol.addObject("/toolbox/side_front");
  if(!smaller) fol.addObject("/toolbox/side_back");
  fol.addObject("screwdriverHandle");
  fol.addObject("screwbox");
  fol.addFact({"table","tableC"});
  fol.addFact({"table","tableL"});
  fol.addFact({"table","tableR"});
  if(!smaller) fol.addAgent("baxterL");
  fol.addAgent("baxterR");
  fol.addAgent("handL");
  fol.addAgent("handR");

  fol.reset_state();
  FILE("z.start.fol") <<fol;
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

//  rnd.clockSeed();

  CooperationDemo demo;

  OptLGP opt(demo.kin, demo.fol);

  opt.run();
  mlr::wait(.1);

  return 0;
}
