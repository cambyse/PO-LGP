/*
 * Feature.cpp
 *
 *  Created on: Nov 9, 2012
 *      Author: robert
 */

#include "Feature.h"

int Feature::field_width[2] = {0,0};
long Feature::id_counter = 0;
const char* ActionFeature::action_strings[5] = { " STAY", "   UP", " DOWN", " LEFT", "RIGHT" };
