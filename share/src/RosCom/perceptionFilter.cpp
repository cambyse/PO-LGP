/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
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

#include <unordered_set>
#include "perceptionFilter.h"

Filter::Filter():Module("filter", -1),
  perceptual_inputs(this, "perceptual_inputs", true),
  object_database(this, "object_database", false)
{}

void Filter::step(){

  perceptual_inputs.writeAccess();
  object_database.writeAccess();

  FilterObjects perceptualInputs = perceptual_inputs();
  FilterObjects objectDatabase = object_database();

  // If empty inputs, do nothing.
  if (perceptualInputs.N == 0 && objectDatabase.N == 0) {
    object_database.deAccess();
    perceptual_inputs.deAccess();
    return;
  }

  FilterObjects filteredInputs;
  FilterObjects DatabaseObjectsOfGivenType, PerceptualInputsOfGivenType;

  // For each type of inputs, run the algorithm.
  for (auto const& type : {FilterObject::FilterObjectType::alvar,
                           FilterObject::FilterObjectType::cluster,
                           FilterObject::FilterObjectType::plane,
                           FilterObject::FilterObjectType::optitrackbody,
                           FilterObject::FilterObjectType::optitrackmarker})
  {
    // Grab the subset from the inputs matching this type
    PerceptualInputsOfGivenType.clear();
    for (uint i = 0; i < perceptualInputs.N; i++) {
      if (perceptualInputs(i)->type == type) {
        PerceptualInputsOfGivenType.append(perceptualInputs(i));
      }
    }

    // Find all matching object of this type
    DatabaseObjectsOfGivenType.clear();
    for (uint i = 0; i < objectDatabase.N; i++) {
      if (objectDatabase(i)->type == type) {
        DatabaseObjectsOfGivenType.append(objectDatabase(i));
      }
    }

    if (PerceptualInputsOfGivenType.N == 0 && DatabaseObjectsOfGivenType.N == 0)
      continue;

    // Create bipartite costs
    costs = createCostMatrix(PerceptualInputsOfGivenType, DatabaseObjectsOfGivenType);

    // Run Hungarian algorithm
    Hungarian ha(costs);

    // Now we have the optimal matching. Assign values.
    FilterObjects assignedObjects = assign(PerceptualInputsOfGivenType, DatabaseObjectsOfGivenType, ha);

    filteredInputs.append(assignedObjects);

  }

  //-- delete objects in percepts and database that are not filtered
  for (FilterObject* fo : perceptual_inputs()) {
    if (!filteredInputs.contains(fo))
      delete fo;
  }
  for (FilterObject* fo : object_database()) {
    if (!filteredInputs.contains(fo))
      delete fo;
  }

  // Set the variable
  object_database() = filteredInputs;
  object_database.deAccess();
  perceptual_inputs.deAccess();
}

FilterObjects Filter::assign(const FilterObjects& perceps, const FilterObjects& database, const Hungarian& ha)
{
  FilterObjects new_objects;
  std::unordered_set<int> matched_ids;

  uint num_old = database.N;
  uint num_new = perceps.N;

  for (uint i = 0; i < ha.starred.dim(0); ++i ) {
    uint col = ha.starred[i]().maxIndex();
    // 3 cases:
    // 1) Existed before, no longer exists. If i > num_new
    // 2) Existed before and still exists. If costs < distannce_threshold
    // 3) Didn't exist before. This happens iff col >= num_old

    // Existed before, doesn't exist now.
    if ( i >= num_new ) {
      //std::cout<< "Existed before, doesn't now." << std::endl;
      FilterObject *new_obj = database(col);
      new_obj->relevance *= relevance_decay_factor;
      new_objects.append(new_obj);
    } else {
      if ( ( col < num_old ) && (costs(i, col) < distance_threshold) ) { // Existed before
        //std::cout<< "Existed before, does now" << std::endl;
        FilterObject *new_obj = perceps(i);
        if (new_obj->type != FilterObject::FilterObjectType::alvar)
          new_obj->id = database(col)->id;
        new_objects.append( new_obj );
      } else { // This didn't exist before. Add it in
        //std::cout<< "Didn't exist before, or not close enough." << std::endl;
        FilterObject *new_obj = perceps(i);
        if (new_obj->type != FilterObject::FilterObjectType::alvar) {
          new_obj->id = maxId;
          maxId++;
        }
        new_objects.append(new_obj);
        //std::cout << "Didn't exist before. Col >= num_old: " << col << ' ' << num_old << std::endl;
      }
    }
    matched_ids.insert(new_objects(i)->id);
    //std::cout << "Assigning \t" << i << "\tMatches:\t" << matched_ids(i).id << "\t Relevance: " << perceps(i).relevance << std::endl;
  }

  // For each of the old objects, update the relevance factor.
  for ( uint i = 0; i < database.N; ++i ) {
    if ( matched_ids.find(database(i)->id) == matched_ids.end() ) {
      FilterObject *new_obj = database(i);
      new_obj->relevance *= relevance_decay_factor;
      new_objects.append(new_obj);      
    }
  }
  FilterObjects cleaned;
  uint count = new_objects.N;
  for ( uint i = 0; i < count; ++i ) {
    if(new_objects(i)->relevance > relevance_threshold) {
      cleaned.append(new_objects(i));
    }
  }

  return cleaned;
}

arr Filter::createCostMatrix(const FilterObjects& newObjects, const FilterObjects& oldObjects)
{
  // First, make a padded out matrix to ensure it is bipartite.
  uint num_new = newObjects.N;
  uint num_old = oldObjects.N;
  uint dims = std::max(num_old, num_new);
  arr costs = ones(dims, dims) * -1.0;

  // Assign costs
  for (uint i=0; i<num_new; ++i) for (uint j=0; j<num_old; ++j) {
    costs(i,j) = newObjects(i)->idMatchingCost(*oldObjects(j));
  }

  // For every element that hasn't been set, set the costs to the max.
  double max_costs = costs.max();

  for (uint i=0; i<dims; ++i) for (uint j=0; j<dims; ++j) {
    if (( i >= num_new ) || ( j >= num_old ))
      costs(i,j) = max_costs;
  }
  return costs;
}
