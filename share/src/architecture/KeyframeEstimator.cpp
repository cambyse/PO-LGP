#include "motion.h"

KeyframeEstimator::KeyframeEstimator():Process("KeyframeEstimator"){
}

KeyframeEstimator::~KeyframeEstimator(){
}

void KeyframeEstimator::open(){
}

void KeyframeEstimator::step(){
    MT::wait(1.);  return;
}

void KeyframeEstimator::close(){
}
