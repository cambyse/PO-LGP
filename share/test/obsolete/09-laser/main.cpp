#define MT_IMPLEMENTATION

#define NIKOLAY

#include <MT/ors.h>
#include <MT/opengl.h>
#include <MT/plot.h>
//#include <MT/top_methods.h>
//#include <MT/top_orsImplementation.cpp>
//#include <MT/top_interface.h>

#include <NJ/UrgInterface.h>

OpenGL gl;
 
 void showPoints(const arr & p){
   int K = p.d0;
   plotData.points.clear();
   plotData.points.resize(K);
   for(int np = 0; np < K; np++){
     plotData.points(np) = ARR(p(np,0),p(np,1),0);
   }
   //gl.watch();
 }
 
 void showAsLines(const arr & p){
   int K = p.d0;
   arr line(K,3);line.setZero();
   for(int np = 0; np < K; np++){
     line(np,0) = p(np,0);
     line(np,1) = p(np,1);
   }
   plotData.lines.append(line);
   gl.text.clr() <<"lines: " << plotData.lines.N ;
   gl.update();
  // gl.watch();
 }
 
 void showResults(const MT::Array<arr>& laserpoints){
   plotData.lines.clear();
   arr po(10,2);po.setZero();//to show origin
   //showPoints(po);
   //gl.watch();
   for(uint i = 0; i < laserpoints.N;i++){
     cout << "total points " << laserpoints(i).d0 << endl;
     //showAsLines(laserpoints(i)/4000.0);
     showPoints(laserpoints(i)/4000.0);
   }
   
   gl.watch();
 }
 
 
 
 MT::Array<arr> Transform3D( MT::Array<arr> scan){
   MT::Array<arr> out3d;
   
   ors::Vector z(1,0,0);
   
   for(uint i = 0; i < scan.N;i++)
   {
     double phi = i*(3.14/6)/60;//divide 30' in 60 parts
     ors::Quaternion R;
     R.setRad(phi,z);
     arr points(scan(i).d0, 3);
     for(uint j=0; j<scan(i).d0;j++){
       ors::Vector x;
       x(0) = scan(i)(j,0); x(1) = scan(i)(j,1); x(2) = 0;//scans always in a xy plane, rotated!!
       x = R*x;
       points[j] = arr(x.v,3);
     }
     out3d.append(points);
   }
   return out3d;
 }
 
 
 void LaserSession(){
   UrgInterface ur;
   ur.init();
     
   for(int i = 0; i < 100; i++){
     arr line;
     ur.scanLine(line);
     MT::Array<arr> temp;
     temp.append(line);
     showResults(temp);
   }
     
   ur.close();
 }
 
 int main(int argn,char **argv){
   gl.add(plotDrawOpenGL,&plotData);
   gl.focus(0,0,.8);
   
   LaserSession();
   return 0;
 }
