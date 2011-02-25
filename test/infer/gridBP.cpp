

void gridBP(arr& post, const arr& evid, const arr& coupling){
  CHECK(evid.nd==3,"");
  uint n=evid.d2; //cardinality of each RV
  uint w=evid.d1; //width of grid
  uint h=evid.d0; //height of grid
  uint i,j,k;
  
  //display
  byteA img(h,w);
  double max;
  for(i=0;i<h;i++) for(j=0;j<w;j++){
    img(i,j)=0;
    max=0.;
    for(k=0;k<n;k++) if(evid(i,j,k)>max){ max=evid(i,j,k); img(i,j)=k; }
  }
  write_ppm(img,"z.ppm");
  MT::wait();

  //-- generate RVs
  VariableList V(h,w);
  for(i=0;i<h;i++) for(j=0;j<w;j++){
    V(i,j) = new Variable(n, STRING("var("<<i<<","<<j<<")"));
  }

  //-- generate factors:
  /*
  FactorList Fhori(h,w-1); //horizontal factors
  FactorList Fvert(h-1,w); //vertical factors
  for(i=0;i<h;i++) for(j=0;j<w;j++){
    if(j<w-1) Fhori(i,j) = new Factor(TUPLE(V(i,j), V(i,j+1)), coupling);
    if(i<h-1) Fvert(i,j) = new Factor(TUPLE(V(i,j), V(i+1,j)), coupling);
  }*/
  Factor Fcoupling(TUPLE(V(0,0), V(0,1)), coupling);
  
  //-- generate messages:
  FactorList MhoriRight(h,w-1),MhoriLeft(h,w-1); //horizontal factors
  FactorList MvertDown(h-1,w),MvertUp(h-1,w); //vertical factors
  for(i=0;i<h;i++) for(j=0;j<w;j++){
    if(j<w-1) MhoriRight(i,j) = new Factor(TUPLE(V(i  ,j+1)));
    if(j<w-1) MhoriLeft(i,j)  = new Factor(TUPLE(V(i  ,j  )));
    if(i<h-1) MvertDown(i,j)  = new Factor(TUPLE(V(i+1,j  )));
    if(i<h-1) MvertUp(i,j)    = new Factor(TUPLE(V(i  ,j  )));
  }

  //-- loop!!
  Factor f,e;
  f.P.resize(n);
  for(;;){
    //-- right-left along the horizontal messages
    
    //RIGHT
    for(i=0;i<h;i++) for(j=0;j<w-1;j++){
      f.P = 1.;
      //if(j<w-1) f.P *= MhoriLeft (i  ,j  )->P;
      if(j)     f.P *= MhoriRight(i  ,j-1)->P;
      if(i)     f.P *= MvertDown (i-1,j  )->P;
      if(i<h-1) f.P *= MvertUp   (i  ,j  )->P;
      f.P *= evid.subDim(i,j);
      normalizeDist(f.P);
      MhoriRight(i,j)->P = coupling*f.P;   //tensorProductMarginal(*MhoriRight(i,j), Fcoupling, f, TUPLE(V(i,j)));
    }

    //LEFT
    for(i=0;i<h;i++) for(j=w-1;j>0;j--){
      f.P = 1.;
      if(j<w-1) f.P *= MhoriLeft (i  ,j  )->P;
      //if(j)     f.P *= MhoriRight(i  ,j-1)->P;
      if(i)     f.P *= MvertDown (i-1,j  )->P;
      if(i<h-1) f.P *= MvertUp   (i  ,j  )->P;
      f.P *= evid.subDim(i,j);
      normalizeDist(f.P);
      MhoriLeft(i,j-1)->P = coupling * f.P; //tensorProductMarginal(*MhoriLeft(i,j-1), Fcoupling, f, TUPLE(V(i,j)));
    }
    
    //DOWN
    for(j=0;j<w;j++) for(i=0;i<h-1;i++){
      f.P = 1.;
      if(j<w-1) f.P *= MhoriLeft (i  ,j  )->P;
      if(j)     f.P *= MhoriRight(i  ,j-1)->P;
      if(i)     f.P *= MvertDown (i-1,j  )->P;
      //if(i<h-1) f.P *= MvertUp   (i  ,j  )->P;
      f.P *= evid.subDim(i,j);
      normalizeDist(f.P);
      MvertDown(i,j)->P = coupling * f.P; //tensorProductMarginal(*MvertDown(i,j), Fcoupling, f, TUPLE(V(i,j)));
    }

    //UP
    for(j=0;j<w;j++) for(i=h-1;i>0;i--){
      f.P = 1.;
      if(j<w-1) f.P *= MhoriLeft (i  ,j  )->P;
      if(j)     f.P *= MhoriRight(i  ,j-1)->P;
      //if(i)     f.P *= MvertDown (i-1,j  )->P;
      if(i<h-1) f.P *= MvertUp   (i  ,j  )->P;
      f.P *= evid.subDim(i,j);
      normalizeDist(f.P);
      MvertUp(i-1,j)->P = coupling*f.P; //tensorProductMarginal(*MvertUp(i-1,j), Fcoupling, f, TUPLE(V(i,j)));
    }

    //get marginals
    post.resizeAs(evid);
    for(i=0;i<h;i++) for(j=0;j<w;j++){
      f.P = 1.;
      if(j<w-1) f.P *= MhoriLeft (i  ,j  )->P;
      if(j)     f.P *= MhoriRight(i  ,j-1)->P;
      if(i)     f.P *= MvertDown (i-1,j  )->P;
      if(i<h-1) f.P *= MvertUp   (i  ,j  )->P;
      f.P *= evid.subDim(i,j);
      normalizeDist(f.P);
      post.subDim(i,j)() = f.P;
    }
    
    //display
    for(i=0;i<h;i++) for(j=0;j<w;j++) img(i,j) = post.subDim(i,j).maxIndex();
    write_ppm(img,"z.ppm");
    MT::wait();
    //break;
  }
  listDelete(MhoriRight);
  listDelete(MhoriLeft);
  listDelete(MvertDown);
  listDelete(MvertUp);
  listDelete(V);
}
