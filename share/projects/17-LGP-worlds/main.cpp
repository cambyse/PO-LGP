#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

#include <LGP/optLGP.h>

void illustrate(){

  OpenGL gl("Red Ball Scenes", 1200, 800);

  uint N=6;
  mlr::Array<mlr::KinematicWorld> K(N);
  for(uint i=0;i<N;i++){
    K(i).init(STRING("problem-0"<<i+1<<".g"));
    K(i).orsDrawMarkers=false;
    gl.addView(i, glStandardScene, NULL);
    gl.addSubView(i, K(i));
    gl.views(i).camera.setDefault();
    gl.views(i).camera.focus(1., 0., .7);

  }
  gl.setSubViewTiles(3,2);

  gl.watch();
}

void solve6(){
  mlr::KinematicWorld K("problem-06.g");
  FOL_World L(FILE("fol.g"));

  //-- prepare logic world
  L.addObject("redBall");
  L.addObject("stick");
  L.addObject("box");
  L.addFact({"pusher", "stickTip"});
  L.addFact({"partOf", "stickTip", "stick"});
  L.addFact({"table","table1"});
  L.addFact({"table","tableR"});
  L.addFact({"table","tableL"});
  L.addFact({"table","box"});
    //    fol.addAgent("pr2L");
  L.addAgent("baxterL");
  L.addAgent("baxterR");
//  L.addAgent("stickTip");
//  L.addAgent("obj1");
    //    fol.addAgent("handL");
    //    fol.addAgent("handR");

  OptLGP lgp(K, L);

  lgp.optFixedSequence("(grasp baxterR stick) \
                       (push stick stickTip redBall box) \
                       (drop redBall box table1) \
                       (place world redBall table1) \
                       (grasp baxterL redBall) \
                       ", true);

/*
                                            (handover baxterR stick baxterL) \
                       (push stick stickTip obj1 table1) \
                       (grasp baxterR obj1) \
                       (place baxterR obj1 tableR) \
                       (place baxterL stick tableL) \
*/


  mlr::wait();

  lgp.renderToVideo();
}

int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

//  illustrate();
  solve6();

  return 0;
}
