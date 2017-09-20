#include "lexicon_TTSR.h"

//need functions that do the following:
// calculate confidence of word
// update association between word and concept element
// choose best location for word
// choose best word for location
// load lexicon from file
// save lexicon to file
// find closest concept element for current location of object
// calculate neighbourhood size from table dimensions

void lexicon::init_lexicon(){
  //large square table
//  tableSize[0] = 0.8;
//  tableSize[1] = 0.8;
  // small rectangular table with width > length
  tableSize[0] = 0.44;
  tableSize[1] = 0.59;
  // small rectangular table with width < length
//  tableSize[0] = 0.59;
//  tableSize[1] = 0.44;
  objectSize = 0.06;
  neighbourhoodSize = sqrt(pow(9,2)+pow(9,2));
  memset(dltTTSR,0,sizeof(dltTTSR));
}

xyIndex lexicon::table_position_to_index(mlr::Vector tpos){
  xyIndex returnIndex;
    returnIndex.x = floor (10.0 * (tpos.x));
  if (returnIndex.x<0)
    returnIndex.x=0;
  if (returnIndex.x>9)
    returnIndex.x=9;
  returnIndex.y = floor (10.0 * (tpos.y));
  if (returnIndex.y<0)
    returnIndex.y=0;
  if (returnIndex.y>9)
    returnIndex.y=9;
  return returnIndex;
}

mlr::Vector lexicon::index_to_table_position(xyIndex index){
  mlr::Vector tpos;
  tpos.x = index.x / 10.0 + 0.05;
  tpos.y = index.y / 10.0 + 0.05;
  return tpos;
}

mlr::Vector lexicon::robot_to_raw_table_position(Roopi& R, mlr::Vector rpos){
  mlr::KinematicWorld kw = R.getK();
  mlr::Body *b1=kw.getBodyByName("base_footprint");
  mlr::Body *b2=kw.getBodyByName("table");
  mlr::Vector tpos;
  arr y,J;
  kw.kinematicsRelPos(y,J,b1,rpos,b2,NoVector);
  tpos.x = y.popFirst();
  tpos.y = y.popFirst();
  return tpos;
}

mlr::Vector lexicon::raw_table_to_robot_position(Roopi& R, mlr::Vector tpos){
  mlr::KinematicWorld kw = R.getK();
  mlr::Body *b1=kw.getBodyByName("table");
  mlr::Body *b2=kw.getBodyByName("base_footprint");
  mlr::Vector rpos;
  arr y,J;
  kw.kinematicsRelPos(y,J,b1,tpos,b2,NoVector);
  rpos.x = y.popFirst();
  rpos.y = y.popFirst();
  return rpos;
}

mlr::Vector lexicon::raw_table_position_to_table_position(mlr::Vector rtpos){
  mlr::Vector tpos;
  tpos.x = (rtpos.y+tableSize[1]/2.0-objectSize) / (tableSize[1]-2.0*objectSize);
  if (tpos.x>1.0){
    tpos.x=1.0;
  }
  if (tpos.x<0.0){
    tpos.x=0.0;
  }
  tpos.y = (rtpos.x+tableSize[0]/2.0-objectSize) / (tableSize[0]-2.0*objectSize);
  if (tpos.y>1.0){
    tpos.y=1.0;
  }
  if (tpos.y<0.0){
    tpos.y=0.0;
  }
  return tpos;
}

mlr::Vector lexicon::table_position_to_raw_table_position(mlr::Vector tpos){
  mlr::Vector rtpos;
  // possibly include object size in these calculations so that doesn't end up off the side of the table?
  rtpos.x = (tpos.y) * (tableSize[0]-2.0*objectSize) - (tableSize[0]/2.0) + objectSize;
  rtpos.y = (tpos.x) * (tableSize[1]-2.0*objectSize) - (tableSize[1]/2.0) + objectSize;
  return rtpos;
}


void lexicon::update_lexicon(Roopi& R, mlr::Vector pos, int word){
  xyIndex index = table_position_to_index(pos);
  dltTTSR[index.x+10*index.y][word] = dltTTSR[index.x+10*index.y][word]+1;
  cout<<"("<<pos.x<<","<<pos.y<<";"<<index.x<<","<<index.y<<")"<<endl;
}

void lexicon::load_lexicon(mlr::String filename){
  ifstream lexiconFile(filename);
  //read each line of file and put into dltTTSR
  //each line is: i index, j index, value
  int a;
  int b;
  int c;
  while (lexiconFile >> a){
    lexiconFile >> b;
    lexiconFile >> c;
    dltTTSR[a][b]=c;
  }
  lexiconFile.close();
}

void lexicon::save_lexicon(mlr::String filename){
  ofstream lexiconFile(filename);
  for (int i=0; i<100; i++){
    for (int j=0; j<5; j++){
      if (dltTTSR[i][j]>0){
        lexiconFile<<i<<" "<<j<<" "<<dltTTSR[i][j]<<endl;
      }
    }
  }
}

int lexicon::choose_object(Roopi& R, int word, std::vector<HRIObject*> objects){
  int returnIndex = -1;
  // for all objects, calculate the confidence of the word at that position
  // return the index of the object with the highest confidence
  double highestConfidence = 0.0;
  for (unsigned int i=0; i<objects.size(); i++){
    mlr::Vector tpos = raw_table_position_to_table_position(robot_to_raw_table_position(R,objects[i]->pos));
    double objConfidence = calculate_confidence(word,tpos);
    if (objConfidence>highestConfidence){
      highestConfidence=objConfidence;
      returnIndex=i;
    }
  }
  return returnIndex;
}

mlr::Vector lexicon::choose_position(Roopi& R,int word){
  // position returned is in raw table coordinates, can be used directly as x and y offset with table as target
  mlr::Vector pos;
  pos.x = 0.0;
  pos.y = 0.0;
  double highestConfidence = 0.0;
  for (int i=0; i<100; i++){
    xyIndex index;
    index.x = i%10;
    index.y = i/10;
    mlr::Vector posFromIndex = index_to_table_position(index);
    double positionConfidence = calculate_confidence(word,posFromIndex);
    if (positionConfidence>highestConfidence){
      highestConfidence = positionConfidence;
      pos = table_position_to_raw_table_position(posFromIndex);
    }
  }
  cout<<"("<<word<<":"<<pos.x<<","<<pos.y<<";"<<highestConfidence<<")"<<endl;
  return pos;
}

double lexicon::calculate_confidence(int word, mlr::Vector pos){
  //pos is in table coordinates, 0.0 to 1.0 in x and y
  double confidence = 0.0;
  double wordAssociations = 0.0;
  double posWordAssociation = 0.0;
  xyIndex posIndex = table_position_to_index(pos);
  for (int i=0; i<100; i++){
    if (dltTTSR[i][word]>0){
      wordAssociations = wordAssociations+dltTTSR[i][word];
      xyIndex index;
      index.x = i%10;
      index.y = i/10;
      double distance = sqrt(pow((posIndex.x-index.x),2)+pow((posIndex.y-index.y),2));
      posWordAssociation = posWordAssociation + dltTTSR[i][word] * ((neighbourhoodSize - distance) / neighbourhoodSize);
    }
  }
  if (wordAssociations>0){
    confidence = posWordAssociation/wordAssociations;
  }
  return confidence;
}

lexicon_TTSR::lexicon_TTSR()
{
}
