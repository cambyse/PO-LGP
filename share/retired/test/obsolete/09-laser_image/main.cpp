#define MLR_IMPLEMENTATION

#include <Core/array.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>

void displayLaserFile(const char *filename){
  uint t,j;
  ifstream is;
  mlr::open(is,filename);
mlr::IOraw = true;
	ofstream os3("frame3d");
  
  mlr::Transformation f;
  arr line;
  arr pointCloud((uint)0,3);
  for(t=0;;t++){
    f.read(is);
    if(!is.good()) break;
    
    line.readTagged(is,"scanline");
    for(j=0; j<line.d0; j++){
      mlr::Vector scanL(line(j,0)/1000.,line(j,1)/1000.,0);
      scanL = f*scanL;
      pointCloud.append(ARR(scanL(0), scanL(1), scanL(2)));
    }
    mlr::Vector x;  f.rot.getX(x);
    mlr::Vector y;  f.rot.getY(y);
    mlr::Vector z;  f.rot.getZ(z);	
    os3 << f.pos << " " << x << " " << y << " " << z  << " " << line.d0 << endl; os3.flush();
  }
  cout <<"loaded " <<pointCloud.d0 <<" points" <<endl;
  mlr::IOraw = true;
  pointCloud >>FILE("data3d");
  OpenGL gl;
  gl.add(glDrawPlot,&plotModule);
  plotPoints(pointCloud);
  plotModule.drawDots = true;
    //tempGL->camera.focus(0,0,.8);
  gl.watch();
  
  //pointCloud.writeRaw(z);
}

int main(int argc, char *argv[]){
//for(int i=0; i < argc; i++) cout<<argv[i]<<endl;

  if (argc == 1)
    displayLaserFile("../../../robot/test/actions/z.laser");
  else
    displayLaserFile(mlr::String(argv[1]));		
  return 0;
}

