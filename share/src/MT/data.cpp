/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include "opengl.h"
#include "data.h"

void Data::loadSpecific(bool test){
  tag=MT::getParameter<MT::String>("dataSet");
  if(tag=="usps"){  loadUSPS("usps-bin.arr", false);  return;  }
  if(tag=="yeast"){
    if(!test) loadSvmMultiLabelData("multilabel/yeast_train.svm", true);
    else      loadSvmMultiLabelData("multilabel/yeast_test.svm", true); return;
  }
  if(tag=="scene"){
    if(!test) loadSvmMultiLabelData("multilabel/scene_train", true);
    else      loadSvmMultiLabelData("multilabel/scene_test", true); return;
  }
  if(tag=="toy")  {  loadToyData("toydata", 1);  return;  }
  HALT("can't load data set " <<tag);
}

void Data::loadToyData(const char *filename, uint labels){
  arr XY;
  MT::load(XY, filename);
  uint s=XY.d1-labels;
  X = XY.sub(0, -1, 0, s-1);
  Y = XY.sub(0, -1, s, -1);
  cout <<MT_HERE <<"read #data=" <<X.d0 <<" #features=" <<X.d1 <<" #classes=" <<Y.d1 <<endl;
}

void Data::loadSvmMultiLabelData(const char *filename, bool inSharePath){
  ifstream is;
  if(inSharePath) MT::open(is, STRING("/home/mtoussai/share/data/" <<filename));
  else            MT::open(is, filename);
  uint l, max=0, idx;
  double f;
  arr features;
  uintA labels;
  MT::Array<uintA> L; //multi label outputs
  uint i;
  for(i=0;; i++){
    features.clear();
    labels.clear();
    is >>l; labels.append(l); if(l>max) max=l;
    if(!is.good()){  is.clear();  break;  }
    while(MT::peerNextChar(is, "")==','){        is >>PARSE(", ") >>l; labels.append(l); if(l>max) max=l; }
    while(MT::peerNextChar(is, " \t\r")!='\n'){  is >>idx >>PARSE(":") >>f; features.append(f);      }
    L.append(labels);
    X.append(features);
    X.reshape(i+1, features.N);
  }
  //cout <<MYtrain <<X <<endl;
  CHECK(L.N==X.d0, "");
  Y.resize(X.d0, max+1); Y.setZero();
  uint j;
  for(i=0; i<X.d0; i++) for(j=0; j<L(i).N; j++)  Y(i, L(i)(j))=1.;
  cout <<MT_HERE <<"read #data=" <<X.d0 <<" #features=" <<X.d1 <<" #classes=" <<Y.d1 <<endl;
}

#if 0
arr X;
MT::load(X, "/home/mtoussai/share/data/USPS-handwritten-digits/usps-ascii.arr");
cout <<X.getDim() <<endl;

byteA tmp;
copy(tmp, X);
tmp.reshape(tmp.d2, tmp.d1, tmp.d0); //due to Matlab conventions...
tensorPermutation(data, tmp, TUP(1, 0, 2));

ofstream os("/home/mtoussai/share/data/USPS-handwritten-digits/usps-bin.arr", std::ios::binary);
data.write(os, NULL, NULL, true, true);
os.close();
#endif

void Data::loadUSPS(const char *filename, bool inSharePath){
  ifstream is;
  if(inSharePath) is.open(STRING("/home/mtoussai/share/data/" <<filename), std::ios::binary);
  else            is.open(filename, std::ios::binary);
  byteA data;
  data.read(is);
  is.close();
  uint dY=data.d1;
  uint dX=data.d2;
  X.resize(data.d0, dY, dX);
  Y.resize(data.d0, dY, dY); Y.setZero();
  uint i, j;
  for(i=0; i<data.N; i++) X.elem(i) = ((double)data.elem(i))/255.;;
  for(i=0; i<Y.d0; i++) for(j=0; j<dY; j++)  Y(i, j, j)=1.;
  X.reshape(data.d0*dY, dX);
  Y.reshape(data.d0*dY, dY);
  cout <<MT_HERE <<"read #data=" <<X.d0 <<" #features=" <<X.d1 <<" #classes=" <<Y.d1 <<endl;
}

void Data::displayInput(uint i, uint height){
  cout <<"input=" <<X[i] <<" output=" <<Y[i] <<endl;
  byteA img(X.d1);
  for(uint j=0; j<X.d1; j++) img(j) = (byte)(255.*X(i, j));
  img.reshape(height, img.N/height);
  OpenGL gl;
  gl.watchImage(~img, true, 10.);
}

void Data::displayAllInputs(){
  for(uint i=0; i<X.d0; i++)  displayInput(i);
}

void Data::reduce(uint dx, uint dy){
  X = X.sub(0, -1, 0, dx-1);
  Y = Y.sub(0, -1, 0, dy-1);
}

void Data::splitTest(Data &test){
  int Ntrain= MT::getParameter<int>("Ntrain", -1);
  int Ntest = MT::getParameter<int>("Ntest" , -1);
  if(Ntrain==-1 && Ntest==-1) return;
  test.X = X.sub(Ntrain, Ntrain+Ntest-1, 0, -1);
  test.Y = Y.sub(Ntrain, Ntrain+Ntest-1, 0, -1);
  X      = X.sub(0, Ntrain-1           , 0, -1);
  Y      = Y.sub(0, Ntrain-1           , 0, -1);
}
