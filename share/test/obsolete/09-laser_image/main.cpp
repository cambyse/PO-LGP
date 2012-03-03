#define MT_IMPLEMENTATION

#include<MT/array.h>
#include<MT/ors.h>
#include<MT/opengl.h>
#include<MT/plot.h>

void displayLaserFile(const char *filename){
  uint t,j;
  ifstream is;
  MT::open(is,filename);
MT::IOraw = true;
	ofstream os3("frame3d");
  
  ors::Transformation f;
  arr line;
  arr pointCloud((uint)0,3);
  for(t=0;;t++){
    f.read(is);
    if(!is.good()) break;
    
    line.readTagged(is,"scanline");
    for(j=0; j<line.d0; j++){
      ors::Vector scanL(line(j,0)/1000.,line(j,1)/1000.,0);
      scanL = f*scanL;
      pointCloud.append(ARR(scanL(0), scanL(1), scanL(2)));
    }
    ors::Vector x;  f.rot.getX(x);
    ors::Vector y;  f.rot.getY(y);
    ors::Vector z;  f.rot.getZ(z);	
    os3 << f.pos << " " << x << " " << y << " " << z  << " " << line.d0 << endl; os3.flush();
  }
  cout <<"loaded " <<pointCloud.d0 <<" points" <<endl;
  MT::IOraw = true;
  MT::save(pointCloud,"data3d");
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
    displayLaserFile(MT::String(argv[1]));		
  return 0;
}

