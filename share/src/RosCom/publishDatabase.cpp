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
#include "publishDatabase.h"
#include <geometry_msgs/PoseArray.h>
#include <object_recognition_msgs/TableArray.h>

#ifdef MLR_ROS_GROOVY
  #include <ar_track_alvar/AlvarMarkers.h>
  namespace ar = ar_track_alvar;
#else // Assuming INDIGO or later
  #include <ar_track_alvar_msgs/AlvarMarkers.h>
  namespace ar = ar_track_alvar_msgs;
#endif


PublishDatabase::PublishDatabase()
  : Thread("PublishDatabase", -1),
    percepts_filtered(this, "percepts_filtered", true),
    nh(NULL)
{}

void PublishDatabase::open(){
  //ros::init(mlr::argc, mlr::argv, "publish_database", ros::init_options::NoSigintHandler);
  if(mlr::getParameter<bool>("useRos", false))
    nh = new ros::NodeHandle;
  if(nh){
    cluster_pub = nh->advertise<visualization_msgs::MarkerArray>("/tabletop/tracked_clusters", 1);
    alvar_pub = nh->advertise<ar::AlvarMarkers>("/tracked_ar_pose_marker", 1);
    plane_pub = nh->advertise<object_recognition_msgs::TableArray>("/tracked_table_array", 1);
    optitrackmarker_pub = nh->advertise<tf::tfMessage>("/tracked_marker_tf", 1);
    optitrackbody_pub = nh->advertise<tf::tfMessage>("/tracked_body_tf", 1);
    plane_marker_pub = nh->advertise<visualization_msgs::MarkerArray>("/tracked_table_markers_array", 1);
  }
}

void PublishDatabase::close()
{
  if(nh){
    //  nh->shutdown();
    delete nh;
  }
}

visualization_msgs::Marker conv_Percept2Marker(const Percept& object)
{
  visualization_msgs::Marker new_marker;
  new_marker.type = visualization_msgs::Marker::POINTS;
  new_marker.points = conv_arr2points( dynamic_cast<const Cluster&>(object).points);
  new_marker.id = object.id;
  new_marker.scale.x = .001;
  new_marker.scale.y = .001;
  new_marker.lifetime = ros::Duration(0.5);
  new_marker.header.stamp = ros::Time(0.);
  new_marker.header.frame_id = dynamic_cast<const Cluster&>(object).frame_id;

  new_marker.color.a = object.relevance;
  new_marker.color.r = (double)((new_marker.id*10000)%97)/97;
  new_marker.color.g = (double)((new_marker.id*10000)%91)/91;
  new_marker.color.b = (double)((new_marker.id*10000)%89)/89;

  return new_marker;
}

object_recognition_msgs::Table conv_Percept2Table(const Percept& object)
{
  const Plane& plane = dynamic_cast<const Plane&>(object);
  object_recognition_msgs::Table new_table;
  new_table.pose = conv_transformation2pose(plane.transform);
  new_table.convex_hull = conv_arr2points(plane.hull.V);
  new_table.header.stamp = ros::Time(0.);
  new_table.header.frame_id = plane.frame_id;

  return new_table;
}

visualization_msgs::Marker conv_Percept2TableMarker(const Percept& object)
{
  visualization_msgs::Marker new_marker;
  const Plane& plane = dynamic_cast<const Plane&>(object);
  new_marker.type = visualization_msgs::Marker::POINTS;
  new_marker.points = conv_arr2points( plane.hull.V );
  new_marker.id = plane.id;
  new_marker.scale.x = .1;
  new_marker.scale.y = .1;
  new_marker.lifetime = ros::Duration(0.5);
  new_marker.header.stamp = ros::Time(0.);
  new_marker.header.frame_id = plane.frame_id;
  new_marker.color.a = plane.relevance;
  new_marker.color.r = 1.0;
  new_marker.color.g = 0;
  new_marker.color.b = 0;
  new_marker.pose = conv_transformation2pose(plane.transform * plane.frame);
  return new_marker;
}

ar::AlvarMarker conv_Percept2Alvar(const Percept& object)
{
  ar::AlvarMarker new_marker;
  new_marker.header.frame_id = dynamic_cast<const Alvar&>(object).frame_id;
  new_marker.pose.pose = conv_transformation2pose(object.transform);
  new_marker.id = object.id;
  return new_marker;
}

geometry_msgs::TransformStamped conv_Percept2OptitrackMarker(const Percept& object)
{
    geometry_msgs::TransformStamped new_marker;
    new_marker.header.frame_id = dynamic_cast<const OptitrackMarker&>(object).frame_id;
    new_marker.transform = conv_transformation2transform(object.transform);
    new_marker.child_frame_id = STRING("optitrackmarker_" << object.id);
    return new_marker;
}

geometry_msgs::TransformStamped conv_Percept2OptitrackBody(const Percept& object)
{
    geometry_msgs::TransformStamped new_marker;
    new_marker.header.frame_id = dynamic_cast<const OptitrackBody&>(object).frame_id;
    new_marker.transform = conv_transformation2transform(object.transform);
    new_marker.child_frame_id = STRING("optitrackbody_" << object.id);
    return new_marker;
}

#if 0 //deprecated, these are now virtual members of percept
void PublishDatabase::syncCluster(const Cluster* cluster)
{
  modelWorld.writeAccess();
  mlr::String cluster_name = STRING("cluster_" << cluster->id);

  mlr::Body *body = modelWorld().getBodyByName(cluster_name, false);
  if (not body) {
    //cout << cluster_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(modelWorld());
    body->name = cluster_name;
    mlr::Shape *shape = new mlr::Shape(modelWorld(), *body);
    shape->name = cluster_name;
    shape->type = mlr::ST_pointCloud;
    shape = new mlr::Shape(modelWorld(), *body);
    shape->name = cluster_name;
    shape->type = mlr::ST_marker;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .2;
    stored_clusters.append(cluster->id);
  }
  body->X = cluster->frame;
  //cluster->frame = body->X;
  body->shapes(0)->mesh.V = cluster->points;

  mlr::Vector cen = body->shapes(0)->mesh.center();
  body->X.addRelativeTranslation(cen);
  body->shapes(0)->rel.rot = body->X.rot;
  body->X.rot.setZero();

  ((Cluster*)cluster)->transform = body->X;
  //((Cluster*)cluster)->mean = ARR(cen.x, cen.y, cen.z);
  /* If we change the mean, we compare the transformed mean to an untransformed mean later...*/
  modelWorld.deAccess();
}

void PublishDatabase::syncAlvar(const Alvar* alvar)
{
  modelWorld.writeAccess();
  mlr::String alvar_name = STRING("alvar_" << alvar->id);

  mlr::Body *body = modelWorld().getBodyByName(alvar_name, false);
  if (not body) {
//    cout << alvar_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(modelWorld());
    body->name = alvar_name;
    mlr::Shape *shape = new mlr::Shape(modelWorld(), *body);
    shape->name = alvar_name;
    shape->type = mlr::ST_marker;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .2;
    stored_alvars.append(alvar->id);
  }

  body->X = alvar->frame * alvar->transform;
  body->shapes.first()->X = body->X;

  modelWorld.deAccess();
}

void PublishDatabase::syncOptitrackBody(const OptitrackBody* optitrackbody)
{
  modelWorld.writeAccess();
  mlr::String optitrackbody_name = STRING("optitrackbody_" << optitrackbody->id);

  mlr::Body *body = modelWorld().getBodyByName(optitrackbody_name, false);
  if (not body) {
    cout << optitrackbody_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(modelWorld());
    body->name = optitrackbody_name;
    mlr::Shape *shape = new mlr::Shape(modelWorld(), *body);
    shape->name = optitrackbody_name;
    shape->type = mlr::ST_marker;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .1;
    stored_optitrackbodies.append(optitrackbody->id);
  }

  body->X = optitrackbody->frame * optitrackbody->transform;
  body->shapes.first()->X = body->X;

  modelWorld.deAccess();
}

void PublishDatabase::syncOptitrackMarker(const OptitrackMarker* optitrackmarker)
{
  modelWorld.writeAccess();
  mlr::String optitrackmarker_name = STRING("optitrackmarker_" << optitrackmarker->id);

  mlr::Body *body = modelWorld().getBodyByName(optitrackmarker_name, false);
  if (not body) {
    cout << optitrackmarker_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(modelWorld());
    body->name = optitrackmarker_name;
    mlr::Shape *shape = new mlr::Shape(modelWorld(), *body);
    shape->name = optitrackmarker_name;
    shape->type = mlr::ST_sphere;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .03;
    stored_optitrackmarkers.append(optitrackmarker->id);
  }

  body->X = optitrackmarker->frame * optitrackmarker->transform;

  //((Alvar*)alvar)->transform = body->X;
  modelWorld.deAccess();
}

void PublishDatabase::syncPlane(const Plane* plane){
  modelWorld.writeAccess();
  mlr::String plane_name = STRING("plane_" << plane->id);

  mlr::Body *body = modelWorld().getBodyByName(plane_name, false);
  if (not body) {
    //cout << plane_name << " does not exist yet; adding it..." << endl;
    body = new mlr::Body(modelWorld());
    body->name = plane_name;
    mlr::Shape *shape = new mlr::Shape(modelWorld(), *body);
    shape->name = plane_name;
    shape->type = mlr::ST_pointCloud;
    shape = new mlr::Shape(modelWorld(), *body);
    shape->name = plane_name;
    shape->type = mlr::ST_marker;
    shape->size[0] = shape->size[1] = shape->size[2] = shape->size[3] = .2;
    stored_planes.append(plane->id);
  }
  body->X = plane->frame * plane->transform;

  //plane->frame = body->X;
  body->shapes(0)->mesh = plane->hull;
//  body->shapes(0)->mesh.makeTriangleFan();

  mlr::Vector cen = body->shapes(0)->mesh.center();
  body->X.addRelativeTranslation(cen);
  body->shapes(0)->rel.rot = body->X.rot;
  body->X.rot.setZero();

//  ((Plane*)plane)->transform = body->X;
  //((plane*)plane)->mean = ARR(cen.x, cen.y, cen.z);
  /* If we change the mean, we compare the transformed mean to an untransformed mean later...*/
  modelWorld.deAccess();
}
#endif

void PublishDatabase::step(){
  int rev = percepts_filtered.writeAccess();

  if (rev == revision)
  {
    percepts_filtered.deAccess();
    return;
  }
  revision = rev;

  PerceptL objectDatabase = percepts_filtered();

  visualization_msgs::MarkerArray cluster_markers, plane_markers;
  object_recognition_msgs::TableArray table_array;
  ar::AlvarMarkers ar_markers;
  tf::tfMessage optitrackmarker_markers;
  tf::tfMessage optitrackbody_markers;

  mlr::Array<uint> new_clusters, new_alvars, new_planes, new_optitrackmarkers, new_optitrackbodies;

  for (uint i = 0; i < objectDatabase.N; i++)
  {
    switch ( objectDatabase(i)->type )
    {
      case Percept::Type::alvar:
      {
        ar::AlvarMarker alvar = conv_Percept2Alvar(*objectDatabase(i));
        ar_markers.markers.push_back(alvar);
        ar_markers.header.frame_id = alvar.header.frame_id;
//        syncAlvar(dynamic_cast<Alvar*>(objectDatabase(i)));
        dynamic_cast<Alvar*>(objectDatabase(i))->syncWith(modelWorld.set());
        new_alvars.append(objectDatabase(i)->id);
        break;
      }
      case Percept::Type::cluster:
      {
        visualization_msgs::Marker marker = conv_Percept2Marker(*objectDatabase(i));
        cluster_markers.markers.push_back(marker);
//        syncCluster(dynamic_cast<Cluster*>(objectDatabase(i)));
        dynamic_cast<Cluster*>(objectDatabase(i))->syncWith(modelWorld.set());
        new_clusters.append(objectDatabase(i)->id);
        break;
      }
      case Percept::Type::optitrackbody:
      {
        geometry_msgs::TransformStamped optitrackbody = conv_Percept2OptitrackBody(*objectDatabase(i));
        optitrackbody_markers.transforms.push_back(optitrackbody);
//        syncOptitrackBody(dynamic_cast<OptitrackBody*>(objectDatabase(i)));
        dynamic_cast<OptitrackBody*>(objectDatabase(i))->syncWith(modelWorld.set());
        new_optitrackbodies.append(objectDatabase(i)->id);
        break;
      }
      case Percept::Type::optitrackmarker:
      {
        geometry_msgs::TransformStamped optitrackmarker = conv_Percept2OptitrackMarker(*objectDatabase(i));
        optitrackmarker_markers.transforms.push_back(optitrackmarker);
//        syncOptitrackMarker(dynamic_cast<OptitrackMarker*>(objectDatabase(i)));
        dynamic_cast<OptitrackMarker*>(objectDatabase(i))->syncWith(modelWorld.set());
        new_optitrackmarkers.append(objectDatabase(i)->id);
        break;
      }
      case Percept::Type::plane:
      {
        object_recognition_msgs::Table table = conv_Percept2Table(*objectDatabase(i));
        visualization_msgs::Marker marker = conv_Percept2TableMarker(*objectDatabase(i));
        plane_markers.markers.push_back(marker);

        table_array.tables.push_back(table);
        table_array.header.frame_id = table.header.frame_id;
//        syncPlane(dynamic_cast<Plane*>(objectDatabase(i)));
        dynamic_cast<Plane*>(objectDatabase(i))->syncWith(modelWorld.set());
        new_planes.append(objectDatabase(i)->id);
        break;
      }
      default:
      {
        NIY;
        break;
      }
    }
  }
  percepts_filtered.deAccess();

  // Publish ROS messages
  if(nh){
    if (cluster_markers.markers.size() > 0)
      cluster_pub.publish(cluster_markers);

    if (ar_markers.markers.size() > 0)
      alvar_pub.publish(ar_markers);

    if (optitrackbody_markers.transforms.size() > 0)
      optitrackbody_pub.publish(optitrackbody_markers);

    if (optitrackmarker_markers.transforms.size() > 0)
      optitrackmarker_pub.publish(optitrackmarker_markers);
 
   if (table_array.tables.size() > 0)
      plane_pub.publish(table_array);

    if (plane_markers.markers.size() > 0)
      plane_marker_pub.publish(plane_markers);

  }


  // Sync the modelWorld
  modelWorld.writeAccess();
  for (uint id : stored_clusters)
  {
    if (new_clusters.contains(id) == 0)
    {
      // Remove ID from the world
      stored_clusters.removeValue(id, false);
      delete modelWorld().getBodyByName(STRING("cluster_" << id));
    }
  }
  for (uint id : stored_alvars)
  {
    if (new_alvars.contains(id) == 0)
    {
      // Remove ID from the world
      stored_alvars.removeValue(id, false);
      delete modelWorld().getBodyByName(STRING("alvar_" << id));
    }
  }
  for (uint id : stored_optitrackmarkers)
  {
    if (new_optitrackmarkers.contains(id) == 0)
    {
      // Remove ID from the world
      stored_optitrackmarkers.removeValue(id, false);
      delete modelWorld().getBodyByName(STRING("optitrackmarker_" << id));
    }
  }
  for (uint id : stored_optitrackbodies)
  {
    if (new_optitrackbodies.contains(id) == 0)
    {
      // Remove ID from the world
      stored_optitrackbodies.removeValue(id, false);
      delete modelWorld().getBodyByName(STRING("optitrackbody_" << id));
    }
  }
  for (uint id : stored_planes)
  {
    if (new_planes.contains(id) == 0)
    {
      // Remove ID from the world
      stored_planes.removeValue(id, false);
      delete modelWorld().getBodyByName(STRING("plane_" << id));
    }
  }
  modelWorld.deAccess();
}
