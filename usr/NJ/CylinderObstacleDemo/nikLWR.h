arr backgram,backinds,backSparse;
uint K = 8;
uint nH = 250;
double sigma = 0.05;
bool bNormal = false;

double lastLikelihood;


double Distance(const arr & x, const arr & y){
  CHECK(x.d0 == y.d0 && y.d1 == x.d1 && x.nd == 1,"error in dist dimensionality");
  double dist; 
  if(x.d0 == 4){//specialdistkernel for 2 stereo images        
    dist = 0;
    dist += 0.75*(x(0)-y(0))*(x(0)-y(0))+0.0000;
    dist += .75*(x(2)-y(2))*(x(2)-y(2)) + 0.9*(x(3)-y(3))*(x(3)-y(3));
    dist += 410.0*( (x(0)-x(2)) - (y(0)-y(2)))*( (x(0)-x(2)) - (y(0)-y(2)));   
  } 
  else
    dist = sqrDistance(x,y);
  return dist;
}

void SortedIndexes(const arr & X, arr& inds, arr & gram){
  if(backgram.N < 1){//hack, 3d coords alwazs generated, picture sim static cached
    gram = arr (X.d0,X.d0),inds = arr(X.d0,X.d0);if(backSparse.N < 1)backSparse = arr(X.d0,nH);
  }
  else
    if(X.d1 == 4){//so perfect backup
    gram = backgram;inds = backinds;
    return;   //cached values
    }else{
      gram = arr(X.d0,X.d0); inds = arr(X.d0,nH);//limit to top 
    }  
    uint i,ii,j;
    if(backgram.N < 1 ){
      double td;
      for(ii = 0; ii < X.d0; ii++)
        for(i = ii; i < X.d0; i++){
        td  = Distance(X[ii],X[i]);
        gram(ii,i)=td;
        gram(i,ii)=td;
        }
               
        for(ii = 0; ii < X.d0; ii++){
          arr orig = gram[ii];//strange memorz errors if done i 1 step
          arr Sorted= orig;
          std::sort(Sorted.p,Sorted.pstop);
        
          arr index = arr(X.d0);
          for ( i = 0; i < orig.d0; i++ )
            for ( j = 0; j < orig.d0; j++ )
              if(orig(i) == Sorted(j))
                index(j) = i;
          inds[ii] = index;
        }
        if(X.d1 == 3) {backSparse = inds.sub(0,X.d0-1,0,nH-1);}//strange memorz errors
    }
    else{//verz special back routine
      gram.setZero();gram = gram + 1000.0;
      arr gram2(X.d0,nH);
      for(ii = 0; ii < X.d0; ii++)
        for(uint i = 0; i < backSparse.d1; i++){
        int tempind = backSparse(ii,i);
        double dist = Distance(X[ii],X[tempind]);
        gram(ii,tempind) = dist;
        gram2(ii,i) = dist;   
        }          
        for(ii = 0; ii < X.d0; ii++){
          arr orig = gram2[ii];
          arr Sorted= orig;
          std::sort(Sorted.p,Sorted.pstop);
          arr index = arr(nH);
          for ( i = 0; i < orig.d0; i++ )
            for ( j = 0; j < orig.d0; j++ )
              if(orig(i) == Sorted(j))
                index(j) = backSparse(ii,i);
          inds[ii] = index;
        }
    }    
    if(X.d1==4){backgram =gram;backinds = inds;}
}


void SortedIndexesbacked(const arr & X, arr& inds, arr & gram){
  if(backgram.N < 1 || X.d1 == 3){//hack, 3d coords alwazs generated, picture sim static cached
    gram = arr (X.d0,X.d0),inds = arr(X.d0,X.d0);
  }
  else{
    gram = backgram;inds = backinds;
    return;   //cached values
  }
  
  uint i,ii,j;
  for(ii = 0; ii < X.d0; ii++)
    for(i = 0; i < X.d0; i++)
      gram(ii,i) = Distance(X[ii],X[i]);
  
  for(ii = 0; ii < X.d0; ii++){
    arr orig = gram[ii];
    arr Sorted= orig;
    std::sort(Sorted.p,Sorted.pstop);        
    arr index = arr(X.d0);
    for ( i = 0; i < orig.d0; i++ )
      for ( j = 0; j < orig.d0; j++ )
        if(orig(i) == Sorted(j))
          index(j) = i;
    inds[ii] = index;
  }    
  if(X.d1==4){backgram =gram;backinds = inds;}
}
  
  arr lastKnb;
  arr IndexSameness(const arr & i1, const arr & i2, const arr & g1, const arr & g2){
    arr ans(3);ans.setZero();
    lastKnb = arr(i1.d0);
    for(uint i = 0; i < i1.d0; i++){//all instances 
      double tmp = 0,s1 = 0, s2 = 0;      
      for(uint j = 1; j <= K; j++)
        for(uint jj = 1; jj <= K; jj++)       
          if(i1(i,j) == i2(i,jj)){
        double k1 = exp(-g1(i,i1(i,j))/sigma);
        double k2 = exp(-g2(i,i2(i,jj))/sigma);
        tmp+=k1*k2;
        s1 += k1*k1;s2 += k2*k2;
        ans(1) = ans(1)+1;
          }
          ans(0) =ans(0)+ tmp/sqrt(s1*s2+0.0000000001);
          ans(2) = ans(2) + tmp;
          lastKnb(i) = tmp;
    }        
    return ans/(double)(i1.d0*K);
  }
  
  //measure sim in x space, predict in y space
  void LWR(const arr & x, const arr & y, arr & yp,arr & index, arr & gram){
    SortedIndexes(x,index,gram); 
    yp = arr(y.d0,y.d1);
    for(uint i = 0; i < x.d0; i++){// all predictions
      double sum = 0;
      yp[i] = 0;
      for(uint j = 1; j <= K; j++){//top neighbours
        int ind = index(i,j);
        double sim = exp(-gram(i,ind)/sigma);
        sum +=sim;
        yp[i] = yp[i]+ sim*y[ind];
    ///  if(x.d1 == 3 && i == 0)cout <<  sim << " vals " << x[i] - x[ind] << " n " << norm(x[i] - x[ind]) << " " ;       
      }
      yp[i] = yp[i]/sum;
    }    
  }
  
  void TestLWR(){
    int s = 200;
    arr x(s,2),y(s,1);
    for(int i = 0; i < s; i++) {
      for(int j = 0; j < 2; j++) 
        x(i,j) = rnd.uni();
      y(i,0) = x(i,0)*x(i,1); 
    }
    arr yp,in,gr;
    LWR(x,y,yp,in,gr);
    //cout << x << endl << endl << y << endl << endl << yp << endl;
    cout << norm(y-yp)/s;  
    return;
    /*arr gram,index;
   
    SortedIndexes(X,index,gram); 
    cout << X << endl << "index or rank " << index << endl << " gram " << gram << endl;
    */
   
   
  }
  
  void krr(arr& alpha,double& b,double& sigma,
           double lambda, double gamma, const arr& x, const arr& t){
  //K     = rbf(lambda, x, x);
             uint n=x.d0;
             arr Ker(n,n);
  //for(i=0;i<n;i++) rbf(K[i](),lambda,x,x[i]);

  //one   = ones(size(t));
  //R     = chol(K + gamma*eye(length(t)));
  //omega = R\(R'\one);
  //nu    = R\(R'\t);   K*nu = t;
             arr one(n),omega,nu,G; one=1.;
             G = Ker + gamma*Identity(n);
             lapack_Ainv_b_sym(omega,G,one);
             lapack_Ainv_b_sym(nu,G,t);

  //delta = 1/max(sum(omega),realmin);
  //b     = sum(nu)*delta;
  //alpha = nu - omega*b;
  //sigma = std(t - K*alpha - b);
             double delta = sum(omega); if(delta>1e-10) delta=1./delta; else delta=1e20;
             b = sum(nu)*delta;
             alpha = nu - omega*b;
             sigma = sqrt(sumOfSqr(t - Ker*alpha - b)/t.N);
           }

           int nPow = 1;
           
           void krr(double gamma, const arr& x, const arr& y, arr & py, arr & params){            
             arr alpha; double b;         
             uint i,j,n=x.d0;
             arr Ker(n,n);
             for(i=0;i<n;i++) 
               for(j=0; j<n;j++)
                 Ker(i,j) = pow( scalarProduct(x[i],x[j])+1.0,nPow); 

             arr one(n),omega,nu,G; one=1.;
             G = Ker + gamma*Identity(n);
             lapack_Ainv_b_sym(omega,G,one);
             lapack_Ainv_b_sym(nu,G,y);

             double delta = sum(omega); if(delta>1e-10) delta=1./delta; else delta=1e20;
             b = sum(nu)*delta;
             alpha = nu - omega*b;
 //cout << sqrt(sumOfSqr(y - Ker*alpha - b)/y.N) << " " << alpha*x << " " << b << endl;//error, alpha*x, b
             py = Ker*alpha + b;
                          
             arr xt;transpose(xt,x);
             arr p1 = xt*alpha;
             params = arr(x.d1+1);
             for(i = 0;i < x.d1; i++)
               params(i) = p1(i,0);
             params(x.d1) = b;
             
             arr inv;
             inverse(inv,G);
             lastLikelihood += scalarProduct(inv,y-mean(y),y-mean(y));//similar to GP likelihood
           }
  
   //add all 2nd order features
           arr Expand2d(const arr & x){
             arr d2(x.d0,x.d1 + x.d1*(x.d1+1)/2);
             for(uint i = 0; i< x.d0; i++){
               for(uint j = 0; j < x.d1; j++ )
                 d2(i,j) = x(i,j);  
               uint br = x.d1;
               for(uint j = 0; j < x.d1; j++ )
                 for(uint jj = j; jj < x.d1; jj++ )
                   d2(i,br++) = (x(i,j)*x(i,jj));//sqrt(fabs   
             }
             return d2;
           }
           
/*          arr Expand2dS(const arr & x){
           arr d2(x.d0,x.d1*(x.d1+1)/2);
           for(uint i = 0; i< x.d0; i++){
                    
           uint br = 0;
               
           for(uint j = 0; j < x.d1; j++ )
           for(uint jj = j; jj < x.d1; jj++ )
           d2(i,br++) = (x(i,j)*x(i,jj));
                             
}
             
           return d2;
}*/
  
           //expect d0 1, d1 3
           arr Exp2dGrad(const arr & x){
             arr grad(3,9);grad.setZero();
             grad(0,0) = 1;grad(1,1) = 1;grad(2,2) = 1;
             
             grad(0,3) = 2.0*x(0,0);
             grad(0,4) = x(0,1);
             grad(0,5) = x(0,2);
             
             
             grad(1,6) = 2.0*x(0,1);
             grad(1,7) = x(0,2);
             grad(1,4) = x(0,0);
                 
             grad(2,5) = x(0,0);    
             grad(2,7) = x(0,1);    
             grad(2,8) = 2.0*x(0,2);
             
             arr gradT; transpose(gradT,grad);
             return gradT;
           }
           
           //we know x->y, but not y->x, need to predict x
           double ReverseMap(const arr & x, const arr & y,const arr & params, arr & grad,bool bVerbose = false){
             grad = arr(x.d0,x.d1);grad.setZero();
             double prerr = 0,prerr2 =0;
             for(uint j = 0; j < y.d0; j++)//each data point
             {
               double diff,pred1;
               for(uint i = 0; i < y.d1; i++)//each dimension, in our case 4 dimensions
               {
                 double pred = scalarProduct(params.sub(i,i,0,8),Expand2d(x.sub(j,j,0,2))) +params(i,9); 
                 if(i == 0)pred1 = pred;
                 if(i==2) prerr2 += fabs(pred1-pred - (y(j,0)-y(j,2)));//*(pred1-pred - (y(j,0)-y(j,2)));
                 diff =  pred -y(j,i);
                 prerr += fabs(diff);//diff
                 arr g1 = 2*diff*params.sub(i,i,0,8);
                 g1 = g1*Exp2dGrad(x.sub(j,j,0,2));
                 grad[j] = grad[j] + g1;
               }             
             }
             if(bVerbose)  cout << "prerr " << prerr/y.d0 << " prer2 " << prerr2/y.d0 << endl;
             
             //cout << "g " << grad << endl << " p  " << params.sub(0,2,0,1) << endl;//<< " x " << x << " y  " << y <<
             
             return prerr/y.d0;
           }
           
           void AbsNorm(const arr & x){
             for(uint i = 0; i < x.d1; i++){
               double n = 0;
               for(uint j = 0; j < x.d0; j++)
                 n += fabs(x(j,i));
                
               cout << "abserr " << i << ": " << n/x.d0 << endl;
             }             
           }
 
         
           
           
           
           void LR(bool bExpand,double gamma,const arr & x, const arr & y, arr & yp,arr & params ){
             lastLikelihood = 0;
             yp = arr(y.d0,y.d1);
             if(bExpand) params = arr(y.d1,x.d1 + x.d1*(x.d1+1)/2+1);//
             else
          //    params = arr(y.d1,x.d1*(x.d1+1)/2+1);
               params = arr (y.d1,x.d1+1);
             
             if(bExpand) nPow = 1;//when we predict coordinates, easier to expand in 2nd order and not use high order polynoms dot product
             else
               MT::getParameter(nPow,"nPow",0);// nPow = 1;//when we tune joint offset stochastic search work with high order polynoms dot product, npow = 3 for simulation             
               
             for(uint i = 0; i < y.d1; i++){
               double abserr= 0.0;
               arr py, tpar;
               if(bExpand)
                 krr(gamma,Expand2d(x),y.sub(0,y.d0-1,i,i),py,tpar);
               else
                 krr(gamma,(x),y.sub(0,y.d0-1,i,i),py,tpar);//Expand2dS
               
               params[i] = tpar;
              
               for(uint j = 0; j< y.d0; j++){
                 yp(j,i) = py(j,0);
                 abserr += fabs(yp(j,i)-y(j,i));
               }
            //   cout << " b " << i << ":" << tpar(tpar.d0-1);
             //  cout << "params for " << i << " :" << tpar << endl;
              // cout <<  " abserr for " << i << ": " << abserr/y.d0 << endl;
             }   
           } 
           
           void LR(bool bExpand,double gamma,const arr & x, const arr & y, arr & yp){
             arr params;
             LR(bExpand,gamma, x,  y,  yp, params );
           }

           void Testkrr(){
             arr x(300,2);arr y(300,2);
             for (int i = 0; i < 300; i++){
               x(i,0) = rnd.uni();x(i,1) = rnd.uni();
               y(i,0) = x(i,0) + 2*x(i,1)*x(i,0) + 0.7;  
               y(i,1) = x(i,0)*x(i,0) + 25*x(i,1) + 0.2; 
             }
             arr py,params;
             for(double i = 0.01; i <= 0.01; i*=2) {
               LR(true,i,x,y,py);
               for(int j = 0; j < 20; j++)
                 cout << endl << " ########### " << py[j] - y[j] << endl;
             }
           }
 
           arr Mean(const arr & x){
             arr m(x.d1);m.setZero();
             for(uint i = 0; i < x.d0; i++)
               m = m + x[i];
   
             return m/(double)x.d0;
           }
 
           arr Cov(const arr & x, const arr & y){
//[n,p] = size(X);
//X = X-ones(n,1)*mean(X);
//Y = X'*X/(n-1);
             arr m1 = Mean(x);
             arr m2 = Mean(y); 
             arr z(x.d0,2);
             for(uint i = 0; i < x.d0; i++){
               z(i,0) = x(i,0)-m1(0);
               z(i,1) = y(i,0)-m2(0); 
             }
             arr zp;transpose(zp,z);
//arr p; scalarProduct(p,z,z);
             return zp*z/(double)(x.d0-1);
           }
 
           double Corr(const arr & x, const arr & y){
             arr cov = Cov(x,y); 
             return cov(0,1)/sqrt(cov(0,0)*cov(1,1));
           }
 
           
           void PrepareNB(uint K,const arr & query, const arr & x, const arr & y, arr & xL, arr & yL, arr & gram){
             gram = arr(x.d0);uint i,j;
             for(i = 0; i < gram.d0; i++)
               gram(i) = Distance(query,x[i]);
                           
             arr orig = gram;//strange memorz errors if done i 1 step
             arr Sorted= orig;
             std::sort(Sorted.p,Sorted.pstop);
             arr index = arr(gram.d0);
             for ( i = 0; i < orig.d0; i++ )
               for ( j = 0; j < orig.d0; j++ )
                 if(orig(i) == Sorted(j))
                   index(j) = i;
          
             xL = arr(K,x.d1);
             yL = arr(K,y.d1);
             
             for(i = 0; i <K;i++ ){
               xL[i] = x[index(i)];//i9gnore first which is the base value ??, i or i+1
               yL[i] = y[index(i)];
             }
          //   cout << "gr " << gram(0) << " btw " << query << " t " << x[0] << endl;
             //cout << "best " << xL[1] << " # " << index(i) << endl;
             gram = Sorted;         ///was bugged before, what I call gram is actually sorted gram    
             //  cout << "xL " << xL << endl << "yL " << yL << endl;
           }
       
             
           arr Smooth(const arr & badX, int nSm = 5){
             arr xsmooth = badX*1.0;
             for(uint i = nSm; i < badX.d0-nSm; i++){//dont forget we ignore first and last entries
               xsmooth[i]= xsmooth[i]*0.0;
               for(int s = -nSm; s <= nSm; s++)
                 xsmooth[i]= xsmooth[i] + badX[i+s]/(nSm*2.0+1);
             }               //xsmooth[i] = (badX[i-2]+badX[i-1]+badX[i]+badX[i+1]+badX[i+2])/5.0;        
             return xsmooth;
           }
           
           arr VerifyLR(const arr & x, const arr & y, const arr & params, bool bVerbose = true){
             arr x2 = Expand2d(x);
             arr paramT; transpose(paramT,params);
             arr vals = x2*paramT.sub(0,x2.d1-1,0,y.d1-1);
             for(uint i = 0; i < x2.d0; i++)
               vals[i]= vals[i] + paramT[x2.d1];
             
             if(bVerbose){
               cout << "verify error norm" << norm(y-vals) << " ";
               AbsNorm(y-vals);
               
               if(y.d0 > 50){
                 cout << "smoother err ";
                 AbsNorm(y-Smooth(vals));
               }
             }
             return vals;
           }
           
          
           
           //ytest2d array, guess is 1 d;!!!!
           arr Predict1Point(const arr & xtrain, const arr & ytrain, const arr & ytest2d,const arr & badX,double & prfit,double & forwer){
             //arr ans = badX[0];
             arr ans = badX*1.0;            
             arr pX,params,grad;
             arr xLocal,yLocal,gram,gram2;
             PrepareNB(nLoc,ytest2d[0],ytrain,xtrain,yLocal,xLocal,gram);//return ans;//nbs in 3d or 4d space ?
             arr tmpparams,xloc2,yloc2;double sumN = 0;
             for(uint j = 0; j < nLoc ; j++){
               PrepareNB(nLoc2,xLocal[j],xtrain,ytrain,xloc2,yloc2,gram2);
               LR(true,0.005,xloc2,yloc2,pX,tmpparams);
               forwer += norm(yloc2-pX);
               if(j == 0)
                 params = tmpparams*exp(-gram(j)/0.03);
               else
                 params = params + tmpparams*exp(-gram(j)/0.03);
               sumN += exp(-gram(j)/0.03);
             }
             params = params/sumN;
             
             Rprop rprop;
             rprop.init(1e-4);
             for(int g = 0; g < nIter2; g++){
               if(g < nIter2-1)
                 ReverseMap(ans, ytest2d,params,  grad);
               else
                 prfit += ReverseMap(ans, ytest2d,params,  grad,false);
               //ans = badX[0]*1.0;
		arr ta = ans[0];	
               rprop.step(ta,grad);//cout << "a " << ans << endl;//" g " << grad << endl;
		ans[0] = ta;               
                //badX[0] = ans;
             }     
             return ans;
           }
           
                      
           //x is 3d, y is 4d usually
           arr ReverseLocalPredict(const arr & xtrain, const arr & ytrain,const arr & xtest, const arr & ytest  ){
             arr p2o,params,grad,pX,params2;//pX is dummy
             LR(true, 0.003,ytrain,xtrain,pX,params2);
             cout << "error of 1 linear model " ;
             arr badX = VerifyLR(ytest,xtest,params2); //get initial badX guesses  
             
             //LR(true,0.001,xtrain,ytrain,pX,params);//params on train set for x2o
             double prfit = 0,forwer = 0;
             
             for(uint i = 0; i < ytest.d0; i++){if(i%20==0)cout << "." ;
              // arr initval = *1.0;
            //   cout << "initval " << initval << endl;
               arr valTest = Predict1Point(xtrain,ytrain,ytest.sub(i,i,0,3),badX.sub(i,i,0,2),prfit,forwer);
               badX[i] = valTest;                             
             /*  arr xLocal,yLocal,gram,gram2;
             //  PrepareNB(nLoc,badX[i],xtrain,ytrain,xLocal,yLocal,gram);
               PrepareNB(nLoc,ytest[i],ytrain,xtrain,yLocal,xLocal,gram);//nbs in 3d or 4d space ?
              // cout << "ytes " << ytest[i] << endl;
               if (true){               
               if(true){
               arr tmpparams,xloc2,yloc2;double sumN = 0;
               for(uint j = 0; j < nLoc ; j++){
               PrepareNB(nLoc2,xLocal[j],xtrain,ytrain,xloc2,yloc2,gram2);
               LR(true,0.005,xloc2,yloc2,pX,tmpparams);//cout << "w " << exp(-gram(j)/0.03) << endl;//cout << "tmpp " << tmpparams.sub(0,2,0,1) << endl;
               forwer += norm(yloc2-pX);
               if(j == 0)
               params = tmpparams*exp(-gram(j)/0.03);
               else
               params = params + tmpparams*exp(-gram(j)/0.03);
               sumN += exp(-gram(j)/0.03);
             }
               params = params/sumN;
             }
               else{               
               LR(true,0.005,xLocal,yLocal,pX,params);   forwer += norm(yLocal-pX);      
             }      
               Rprop rprop;
               rprop.init(1e-4);
               for(int g = 0; g < nIter2; g++){
               if(g < nIter2-1)
               ReverseMap(badX.sub(i,i,0,2), ytest.sub(i,i,0,3),params,  grad);
               else
               prfit += ReverseMap(badX.sub(i,i,0,2), ytest.sub(i,i,0,3),params,  grad,false);
               arr tmp = badX[i];
               rprop.step(tmp,grad);
               badX[i] = tmp;
             }
             }
               else{//use 4d->3d regresion, siginificantlz worse
               arr tmpparams,xloc2,yloc2;double sumN = 0;
               for(uint j = 0; j < nLoc ; j++){
               PrepareNB(50,yLocal[j],ytrain,xtrain,yloc2,xloc2,gram2);
               LR(true,0.005,yloc2,xloc2,pX,tmpparams);
               forwer += norm(xloc2-pX);
               arr pred = VerifyLR(ytest.sub(i,i,0,3),xtest.sub(i,i,0,2),tmpparams,false);
               if(j == 0)
               badX[i] = pred*exp(-gram(j)/0.03);
               else
               badX[i] =badX[i] + pred*exp(-gram(j)/0.03);
               sumN += exp(-gram(j)/0.03);
             }
               badX[i] = badX[i]/sumN;
                 
             }
               cout << "after " << badX[i] -  valTest << endl;
             */
             }
             cout << endl << "finished reverse fitting" << endl << "error no smoothing " ;
             AbsNorm(xtest-badX);
             arr sm = Smooth(badX,3);
             cout << "error smoothed " ;
             AbsNorm(xtest-sm);
             cout << "prfit: " << prfit/xtest.d0 << " forwer: " << forwer/(xtest.d0*nLoc);
             std::ofstream fil1;    MT::open(fil1,"kd/xtest");             
             arr plotdata(xtest.d0,10);
             for(uint i = 0; i < plotdata.d0; i++){
               plotdata(i,0) = i;
               plotdata(i,1) = xtest(i,0); 
               plotdata(i,2) = xtest(i,1); 
               plotdata(i,3) = xtest(i,2); 
               plotdata(i,4) = badX(i,0); 
               plotdata(i,5) = badX(i,1); 
               plotdata(i,6) = badX(i,2); 
               plotdata(i,7) = sm(i,0); 
               plotdata(i,8) = sm(i,1); 
               plotdata(i,9) = sm(i,2); 
             }
             fil1 << plotdata;
            
             if(false){
               std::ofstream fil;
               MT::open(fil,"z.trana");
               fil << plotdata;
               gnuplot("plot 'z.trana' us 1:2 title  ' xre ', 'z.trana' us 1:3 title 'yre','z.trana' us 1:4 title 'zre'             ,'z.trana' us 1:5 title 'xpr','z.trana' us 1:6 title 'ypr','z.trana' us 1:7 title 'zpr'             ,'z.trana' us 1:8 title 'xsm','z.trana' us 1:9 title 'ysm','z.trana' us 1:9 title 'zsm' ");
             }
             return badX;
           }
             
           arr lastInv;
           void Likelihood(double gamma, const arr& x, const arr& y, double & val, arr & grad){                          
             arr inv; 
             if(lastInv.N < 2 || false){//turn speed off when using check gradient
               cout << " ### " ;
             uint i,j,n=x.d0;
             arr Ker(n,n);
             for(i=0;i<n;i++) 
               for(j=0; j<n;j++)
                 Ker(i,j) = pow( scalarProduct(x[i],x[j])+1.0,nPow); 
             arr G = Ker + gamma*Identity(n);                        
            
             inverse(inv,G);
             lastInv = inv;
             }else
               inv = lastInv;
             
             arr ym = y-mean(y);             
             val = scalarProduct(inv,ym,ym);//similar to GP likelihood Rasmussen Chapter 5
             
             ym = inv*ym;//precompute
             grad = x*0.0;
             arr Ai = inv*0.0;
             for(uint i = 0; i < x.d0; i++)
               for(uint j = 0; j < x.d0; j++)
                 Ai(i,j) = nPow*pow( scalarProduct(x[i],x[j])+1.0,nPow-1);
             arr At = inv*0.0;
                
             for(uint i = 0; i < x.d0; i++){
               At.setZero();//zero values remain same for j
               for(uint j = 0; j < x.d1; j++){
                   
                 /*for(uint ii = 0; ii < x.d0; ii++)
                   for(uint jj = 0; jj < x.d0; jj++){
                   if(ii==i && jj!=i)
                     At(ii,jj) = Ai(ii,jj)*x(jj,j); 
                  else if(ii!=i && jj==i)
                     At(ii,jj) = Ai(ii,jj)*x(ii,j);
                  else if(ii==i && jj==i)
                     At(ii,jj) = Ai(ii,jj)*2*x(ii,j); 
                   }*/
                 for(uint k = 0; k < x.d0; k++){
                  At(i,k) = Ai(i,k)*x(k,j);
                  At(k,i) = Ai(k,i)*x(k,j);                
                 }                 
                 At(i,i) = Ai(i,i)*2*x(i,j);
                 grad(i,j) = -scalarProduct(At,ym,ym);
               }
             }
           }
             
           void   LikelihoodM(double gamma,const arr & x, const arr y,double & val,arr & grad){
             nPow = 3;
             val = 0;
             grad = x*0.0;
             lastInv = arr(1,1);
             for(uint i = 0; i < y.d1-1; i++){//hacks for speed, ignore last dimension since it is same as second
               double tv= 0.0;
               arr tgr;              
               Likelihood(gamma,x,y.sub(0,y.d0-1,i,i),tv,tgr);
               val +=tv;
               grad +=tgr;
             }
           }    
             
