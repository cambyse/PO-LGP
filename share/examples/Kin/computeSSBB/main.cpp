#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <Core/graph.h>

void col(void*){
  glColor(.5, .0, .0, .3);
}

void computeSSBB() {
  mlr::String file=mlr::getParameter<mlr::String>("file",STRING("test.ors"));
  if(mlr::argc==2 && mlr::argv[1][0]!='-') file=mlr::argv[1];
  cout <<"opening file `" <<file <<"'" <<endl;

  Graph G(file.p);

  OpenGL gl;
  gl.add(glStandardScene);
  mlr::Mesh m, m2;
  gl.add(m);
  gl.add(col);
  gl.add(m2);

  StringA done;
  for(Node *n:G){
    if(n->isGraph()){
      Graph& g = n->graph();
      if(n->keys(1)=="visual" && g["mesh"]){
        mlr::FileToken& f = g["mesh"]->get<mlr::FileToken>();
        if(!done.contains(f.name)){
          cout <<"shape " <<*g["mesh"] <<endl;

          m.readStlFile(f);
//          gl.watch();
          arr x;
          mlr::Transformation t;
          m2.computeOptimalSSBox(x, t, m.V, 2);
          gl.watch();
          //cleanup:
//          for(uint i=0;i<3;i++) if(x(i)<2e-3){x(3) += x(i); x(i)=0.; } //thin
          for(uint i=0;i<3;i++) if(fabs(t.pos(i))<1e-4) t.pos(i)=0.;
          for(uint i=1;i<=3;i++) if(fabs(t.rot(i))<1e-3) t.rot(i)=0.;
          t.rot.normalize();
          m2.setSSBox(x(0), x(1), x(2), x(3));
          t.applyOnPointArray(m2.V);
          gl.watch();

          g.newNode<arr>({"ssbb"},{}, x({0,3}));
          g.newNode<mlr::Transformation>({"ssbb_rel"},{}, t);
          cout <<g <<endl;

          done.append(f.name);
        }
      }
    }
  }

  FILE("z.g") <<G;

  mlr::KinematicWorld K;

}

void postProcess() {
  Graph G("z.g");

  std::map<mlr::String, Graph*> map;

  OpenGL gl;
  gl.add(glStandardScene);
  mlr::Mesh m, m2;
  gl.add(m);
  gl.add(col);
  gl.add(m2);

  for(Node *n:G){
    if(n->isGraph()){
      Graph& g = n->graph();
      if(g["contact"] && g["mesh"]){
        mlr::FileToken& f = g.get<mlr::FileToken>("mesh");
        arr x;
        mlr::Transformation t;
        if(!g["ssbb"]){
          Graph *p = map[f.name];
          CHECK(p,"");
          x = p->get<arr>("ssbb");
          t = p->get<mlr::Transformation>("ssbb_rel");
        }else{
          map.insert({f.name, &g});
          x = g.get<arr>("ssbb");
          t = g.get<mlr::Transformation>("ssbb_rel");
        }
        mlr::Transformation &rel = g.get<mlr::Transformation>("rel");
        m.readFile(f.name.p);
        m2.setSSBox(2.*(x(0)), 2.*(x(1)), 2.*(x(2)), x(3));

        cout <<"x=" <<x <<endl;


        rel.addRelativeTranslation(-m.getMeanVertex());
        rel.applyOnPointArray(m.V);

        rel.appendTransformation(t);
        rel.applyOnPointArray(m2.V);
        gl.watch();

//        rel = rel*t;
        g.get<mlr::String>("type") = "ST_ssBox";
        g.get<arr>("size") = ARR(2.*(x(0)+x(3)), 2.*(x(1)+x(3)), 2.*(x(2)+x(3)), x(3));
        delete g["mesh"];
//        delete g["ssbb"];
//        delete g["ssbb_rel"];
        delete g["rel_includes_mesh_center"];
      }
    }
  }
  FILE("z2.g") <<G;
}


int MAIN(int argc,char **argv){
  mlr::initCmdLine(argc, argv);

  computeSSBB();
//  postProcess();

  return 0;
}
