#include <Core/array.h>
#include <extern/GCO/GCoptimization.h>

void TEST(GCO) {
  int width=100;
  int height=100;
  int num_pixels=width*height;
  int num_labels=10;

  intA result(num_pixels);   // stores result of optimization

  // first set up the array for data costs
  intA data(num_pixels,num_labels);
  for(uint i=0; i<data.d0;i++)
    for(uint l=0; l<data.d1;l++)
      if(i<25){
	if(l==0) data(i,l)=0; else data(i,l)=10;
      }else{
	if(l==5) data(i,l)=0; else data(i,l)=10;
      }
  // next set up the array for smooth costs
  intA smooth(num_labels,num_labels);
  for(uint l1=0; l1<smooth.d0; l1++)
    for(uint l2=0; l2 <smooth.d1; l2++)
      smooth(l1,l2) = ((l1-l2)*(l1-l2) <= 4) ? (l1-l2)*(l1-l2) : 4;


  try{
    GCoptimizationGridGraph *gc = new GCoptimizationGridGraph(width,height,num_labels);
    gc->setDataCost(data.p);
    gc->setSmoothCost(smooth.p);
    for(uint k=0;k<10;k++){
      cout <<"k=" <<k <<" energy= " <<gc->compute_energy() <<endl;
      gc->expansion(1);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
      cout <<"k=" <<k <<" energy= " <<gc->compute_energy() <<endl;
      gc->swap(1);
    }
    //    cout <<"\nAfter optimization energy is " <<gc->compute_energy() <<endl;

    for(uint i=0; i<result.N; i++) result(i) = gc->whatLabel(i);
    
    delete gc;
  }catch (GCException e){
    e.Report();
  }

}


int MAIN(int argc,char** argv){

  testGCO();

  return 0;
}
