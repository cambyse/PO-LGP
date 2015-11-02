#include <Core/util_t.h>
#include <Gui/opengl.h>
#include <Optim/search.h>

#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors_swift.h>
#include "../src/motion_factory.h"
#include "../src/ikmo_bretl.h"


void run() {
  rnd.seed(2);
  bool visTest = mlr::getParameter<uint>("visTest");
  uint verbose = mlr::getParameter<uint>("verbose");

  /// create some motion scenes
  MotionFactory* mf = new MotionFactory();
  mlr::Array<Scene > trainScenes;
  mlr::Array<Scene > testScenes;
  mlr::Array<CostWeight> weights;
  mf->costScale=1e2;
  mf->createScenes(mlr::getParameter<uint>("scene"),trainScenes,testScenes,weights);
  arr param = zeros(3,1);param.flatten();
  arr x;

  IKMO ikmo(trainScenes,weights,param.d0,mf->costScale);
  param = ARR(0.00128167, 22.4908, 22.744, 17.7858 ,3.04831, 1.3858, 0.899768);

//  param = param/length(param)*mf->costScale;
////  cout << "Learned param: " <<param << endl;
//  cout << "Reference param: " <<trainScenes(0).paramRef << endl;
//  mf->execMotion(ikmo,trainScenes(0),param,true,0,x);
//  cout << sum((x-trainScenes(0).xDem)%(x-trainScenes(0).xDem)) << endl;

//  param = ARR(0.372539, 4438.11,7240.43, 3952.09, 3449.62, 369.766, 473.039);
//  param = param/length(param)*mf->costScale;
//  mf->execMotion(ikmo,trainScenes(0),param,true,0,x);
//  cout << sum((x-trainScenes(0).xDem)%(x-trainScenes(0).xDem)) << endl;

//  return;

  SearchCMA cma;
  int cmaD = trainScenes.last().paramRef.d0;
  int cmaMu = -1;
  int cmaLambda = -1;
  double cmaLo = 20.;
  double cmaHi = 30.;

  mlr::timerStart();

//  cma.init(cmaD,cmaMu,cmaLambda,cmaLo,cmaHi);

//  param = ARR(0.111264, 72.6097 ,24.1469, 41.017 ,31.9636, 5.92353, 31.0656);
//  param = ARR(0.111264, 40.6097 ,40.1469, 41.017 ,40.9636, 40.92353, 40.0656);
  param = ones(cmaD,1)*5.;param.flatten();//ARR(0.00180892, 16.7261, 89.5033, 21.8877, 13.7254, 1.78447, 1.39885);
//  param = trainScenes.last().paramRef;
  param = ARR(  0.00115659, 22.2208, 21.7075, 11.2698, 1.76874, 1.2577, 0.380254);
  // = ARR(  0.00179734, 1.17867, 0.271111, 0.364266, 0.278265, 0.306423, 0.379597);//20.*ones(param.d0);
  //  paramStd = ARR( 2.33471e-05, 0.0550647, 0.0149182 ,0.0139366, 0.0117777, 0.0110984, 0.0196474 );
  arr paramStd = ARR( 0.000126209, 0.0460171, 0.181301, 0.134023, 0.0482699, 0.055595, 0.471794 );

//  double paramStd=.5;
//  TIME = 1432.52 + 1468.42+1353.72+1431+1431
  cma.init(cmaD,cmaMu,cmaLambda,param,paramStd);

  arr samples, values;

  double min_value = 1e3;
  double max_value = -1.;
  double costs;
  for(uint t=0;t<1000;t++){
    cma.step(samples, values);

    for(uint i=0;i<samples.d0;i++) {
      param = samples[i];

      //check if param is valid
      bool valid = true;
      for (uint j=0;j<param.d0;j++){
        if (param(j)<0) {
          valid = false;
        }
      }

      if (valid){
        mf->execMotion(ikmo,trainScenes(0),param,false,0,x);
        values(i) = sum((x-trainScenes(0).xDem)%(x-trainScenes(0).xDem));
      } else{
        values(i) = 2.;
      }
      min_value=min(ARR(min_value,values.min()));
      max_value=max(ARR(max_value,values.max()));
    }
    cout << "t "<< endl;
    cout << "samples " << samples << endl;
    cout << "values " << values << endl;
    arr paramStdd(samples.d1);
    for (uint k=0;k<samples.d1;k++){
      arr tmp = ~samples;
      paramStdd(k) = sqrt(var(tmp[k]));
    }
    cout << "paramStd " << paramStdd << endl;
    cout << "TIME: " << mlr::timerRead() << endl;


// compute variance of samples

  }


  param = param/length(param)*mf->costScale;
  cout << "Learned param: " <<param << endl;
  cout << "Reference param: " <<trainScenes(0).paramRef << endl;
  mf->execMotion(ikmo,trainScenes(0),param,false,0,x);
  /// create ikmo problem
//  arr param = trainScenes(0).paramRef;
//  arr param = fabs(randn(trainScenes(0).paramRef.d0,1)); param.flatten();
//  arr param = zeros(trainScenes(0).paramRef.d0,1); param.flatten();
//  param.append(zeros(3));
//  param =1e-7;
//  param(0)=-1;
//  param(0)=1e1;
//  param = param/sum(param)*mf->costScale;


  return;
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  run();
  return 0;
}
