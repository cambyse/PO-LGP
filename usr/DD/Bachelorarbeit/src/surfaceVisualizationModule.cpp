#include "surfaceVisualizationModule.h"
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <Gui/color.h>
#include <Control/TaskControllerModule.h>
#include <limits>

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "objectGenerator.h"


SurfaceGeodesicPathModule::SurfaceGeodesicPathModule()
  : Thread("surfaceGeodesicPathModule", 0.5)
  , surfaceModelObject(NULL, "surfaceModelObject")
  , pathStart(this, "pathStart")
  , pathTarget(this, "pathTarget")
  , shortestPath(this, "shortestPath")
  , realTargetOnSurface(this, "realTargetOnSurface"){}

arr SurfaceGeodesicPathModule::getRealTargetOnSurface() {
  return realTargetOnSurface.get()();
}

arr SurfaceGeodesicPathModule::getPath() {
  return shortestPath.get()();
}

void SurfaceGeodesicPathModule::setStart(arr start) {
  pathStart.set() = start;
}

void SurfaceGeodesicPathModule::setTarget(arr target) {
  pathTarget.set() = target;
}

void SurfaceGeodesicPathModule::step() {
  ors::Mesh me = surfaceModelObject.get()->mesh;
  //arr varianceGradients = surfaceModelObject.get()->varianceGradientsOnSurface;
  arr variance = surfaceModelObject.get()->varianceOnSurface;
  arr start = pathStart.get()();
  arr target = pathTarget.get()();

  arr realTarget;
  arr path = SurfaceModelObject::computeGeodesicEuklideanPathOnSurface(start, target, me, NoArr, realTarget);
  //arr path = SurfaceModelObject::computeGeodesicVariancePathOnSurface(start, target, me, variance, NoArr, realTarget);

  path = surfaceModelObject.set()->smoothGeodesicPathWithKOMO(path);

  realTargetOnSurface.set() = realTarget;
  shortestPath.set() = path;


  /*typedef std::map<std::pair<uint, uint>, double> Weights;
  typedef std::pair<uint, uint> Edge;
  Weights W;

  for(uint i = 0; i < me.T.d0; i++) {
    mlr::Array<uint> T = me.T[i];
    for(uint j = 0; j <= 2; j++) {
      Edge edgeForward(T(j),T((j+1)%3));
      Edge edgeBackward(T((j+1)%3),T(j));
      if(W.find(edgeForward) == W.end()) {
        double distance = length(me.V[edgeForward.first]-me.V[edgeForward.second]);
        //distance = distance*(variance(edgeForward.first)+variance(edgeForward.second))/2.0;
        //distance = 1.0/distance;

        //arr ve = me.V[edgeForward.first]-me.V[edgeForward.second];
        //arr gr = varianceGradients[edgeForward.first];
        //double l = 10.5;
        //double distance = sqrt((~ve*(eye(3) + l*gr*~gr)*ve).scalar());
        W.insert(Weights::value_type(edgeForward, distance));
        //W.insert(Weights::value_type(edgeBackward, distance));
      }
    }
  }

  typedef boost::adjacency_list <boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, double> >   DijkstraGraph;
  typedef boost::graph_traits<DijkstraGraph>::vertex_descriptor Node;

  DijkstraGraph G(me.V.d0);
  for(const auto& w : W) {
    Weights::value_type e = w;
    boost::add_edge(e.first.first, e.first.second, e.second, G);
  }

  std::vector<Node> P(me.V.d0);
  std::vector<double> D(me.V.d0,std::numeric_limits<double>::max());

  uint startIndex = 0, targetIndex = 0;
  for(uint i = 0; i < me.V.d0; i++) {
    if(length(me.V[i]-start) < length(me.V[startIndex]-start)) {
      startIndex = i;
    }
    if(length(me.V[i]-target) < length(me.V[targetIndex]-target)) {
      targetIndex = i;
    }
  }
  realTargetOnSurface.set() = me.V[targetIndex];
  boost::dijkstra_shortest_paths(G, startIndex, boost::predecessor_map(&P[0])/*.distance_map(&D[0])*//*);
  /*arr path;
  path.append(~me.V[targetIndex]);
  uint currentNodeIndex = targetIndex;
  while(true) {
    uint pre = P[currentNodeIndex];
    path.append(~me.V[pre]);
    if(pre == startIndex) break;
    currentNodeIndex = pre;
  }
  path.reverseRows();
  shortestPath.set() = path;

/*
  //Here starts the specific code for that library. I would like to change that library to a better one (boost dijkstra uses a fibonacci heap, much better)
  std::vector<double> points = conv_arr2stdvec(me.V);
  std::vector<uint> faces = conv_arr2stdvec(me.T);

  geodesic::Mesh geodesicMesh;
  geodesicMesh.initialize_mesh_data(points, faces);
  geodesic::GeodesicAlgorithmDijkstra algorithm(&geodesicMesh);
  uint startIndex = 0, targetIndex = 0;
  for(uint i = 0; i < geodesicMesh.vertices().size(); i++) {
    auto& geoVertex = geodesicMesh.vertices()[i];
    arr vertex = ARR(geoVertex.x(), geoVertex.y(), geoVertex.z());
    if(length(vertex-start) < length(ARR(geodesicMesh.vertices()[startIndex].x(), geodesicMesh.vertices()[startIndex].y(), geodesicMesh.vertices()[startIndex].z())-start)) {
      startIndex = i;

    }
    if(length(vertex-target) < length(ARR(geodesicMesh.vertices()[targetIndex].x(), geodesicMesh.vertices()[targetIndex].y(), geodesicMesh.vertices()[targetIndex].z())-target)) {
      targetIndex = i;
    }
  }

  geodesic::SurfacePoint sourceVertex(&geodesicMesh.vertices()[startIndex]);
  geodesic::SurfacePoint targetVertex(&geodesicMesh.vertices()[targetIndex]);
  realTargetOnSurface.set() = ARR(targetVertex.x(), targetVertex.y(), targetVertex.z());

  std::vector<geodesic::SurfacePoint> path;
  algorithm.geodesic(sourceVertex, targetVertex, path);
  cout << "yeaah" << endl;

  arr shortestPathA;
  for(uint i = path.size(); i > 0; i--) {
    shortestPathA.append(~ARR(path[i-1].x(), path[i-1].y(), path[i-1].z()));
  }
  shortestPath.set() = shortestPathA;
  */
}


//===================================================================================================

SurfaceSimilarityModule::SurfaceSimilarityModule()
  : Thread("surfaceSimilarityModule", 0.2)
  , surfaceModelObject(NULL, "surfaceModelObject")
  , surfaceSimilarityMeasure(this, "surfaceSimilarityMeasure")
  , trueSurfaceMesh(NULL, "trueSurfaceMesh") {}

void SurfaceSimilarityModule::step() {
  ors::Mesh mesh = trueSurfaceMesh.get()();
  SurfaceModelObject o = surfaceModelObject.get()();
  double s = o.calculateMeshDistance(mesh);
  surfaceSimilarityMeasure.set() = s;
}

//===================================================================================================


SurfaceVisualizationModule::SurfaceVisualizationModule(Roopi& roopi)
  : Thread("surfaceVisualisation", 0.2)
  , roopi(roopi)
  , gpSurface(NULL, "gpSurface")
  , trueSurfaceMesh(NULL, "trueSurfaceMesh")
  , surfaceModelObject(this, "surfaceModelObject")
  , surfaceSimilarityMeasure(NULL, "surfaceSimilarityMeasure") {
  numberOfIts = 0;
}

void changeColor(void*){  orsDrawColors=false; glColor(.5, 1., .5, .7); }
void changeColor2(void*){  orsDrawColors=true; orsDrawAlpha=1.; }

void SurfaceVisualizationModule::open() {
  OrsPoseViewer* viewer = getThread<OrsPoseViewer>("OrsPoseViewer");
  ors::Shape* trueSurfaceShape = viewer->copies.first()->getShapeByName("trueShape");
  arr posTrueSurface = conv_vec2arr(trueSurfaceShape->X.pos);
  trueSurfaceMesh.writeAccess();
  trueSurfaceMesh() = trueSurfaceShape->mesh;
  for(uint i = 0; i < trueSurfaceShape->mesh.V.d0; i++) {
    trueSurfaceMesh().V[i] += posTrueSurface;
  }
  trueSurfaceMesh.deAccess();

  //me = new ors::Mesh;
  //world.copy(*viewer->copies.first(), true);
  gl.add(glStandardScene);
  gl.camera.setDefault();
  gl.add(me);
  //gl.add(glDrawPlot, &plotModule);
  gl.update("EstimatedSurface", false, false, true);
}

void SurfaceVisualizationModule::step() {
  surfaceModelObject.writeAccess();
  surfaceModelObject().gp = gpSurface.get()();
  if(surfaceModelObject().gp.X.d0) {
    surfaceModelObject().calculateSurface();
    //surfaceModelObject().mesh.fuseNearVertices();
    if(surfaceModelObject().mesh.V.d0) {
      surfaceModelObject().calculateVariance();
      surfaceModelObject().plotVarianceOnSurface();
      me.V = surfaceModelObject().mesh.V;
      me.T = surfaceModelObject().mesh.T;
      me.C = surfaceModelObject().mesh.C;
      surfaceModelObject().calculateGradientsOnSurface();
      //surfaceModelObject().calculateVarianceGradientOnSurface();
      surfaceModelObject.deAccess();
      numberOfIts++;
      //cout << "#It surfaceMesh: " << numberOfIts << endl;
      double uncertaintyMeasure = surfaceModelObject.get()->calculateVarianceMeasure();

      if(uncertaintyMeasure < 0.05) {
        Access_typed<uint> maxIt(NULL,"maxIt");
        maxIt.set()() = 0;
        cout << "Uncertainty is smaller than 0.1, yehaa!!" << endl;
      }

      surfaceSimilarityModule.threadStep();

      double meshDistance = surfaceSimilarityMeasure.get()();//0.0;//surfaceObject.calculateMeshDistance(trueSurfaceMesh);

      gl.update(STRING("EstimatedSurface with Uncertainty " << uncertaintyMeasure << " . MeshDistance: " << meshDistance), false, false, false);
    } else {
      surfaceModelObject.deAccess();
    }
  } else {
    surfaceModelObject.deAccess();
  }

}

void SurfaceVisualizationModule::close() {}

//===================================================================

PathAndGradientVisualizationModule::PathAndGradientVisualizationModule(Roopi& roopi)
  : Thread("pathAndGradientVisualisation", 0.1)
  , roopi(roopi)
  , gpSurface(NULL, "gpSurface")
  , surfaceModelObject(NULL, "surfaceModelObject")
  , gradReference(NULL, "gradReference")
  , shortestPath(NULL, "shortestPath") {}

void PathAndGradientVisualizationModule::open() {
  viewer = getThread<OrsPoseViewer>("OrsPoseViewer");
}

void PathAndGradientVisualizationModule::step() {
  gpCopy = gpSurface.get()();
  if(gpCopy.X.d0 > 0) {
    TaskMap_Default posMap(posTMT, roopi.tcm()->modelWorld.get()(), "endeffR");
    arr pos;
    posMap.phi(pos, NoArr, roopi.tcm()->modelWorld.get()());

    arr gradGP, gradV, X, Y;
    //gpCopy.gradient(gradGP, pos);
    //gpCopy.gradientV(gradV, pos);
    gradGP = gpCopy.gradient(pos);
    gradV = gpCopy.gradientVariance(pos);
    X = gpCopy.X;
    Y = gpCopy.Y;

    gradV.reshapeFlat();
    double lGradGP = length(gradGP);
    arr gradVT = gradV - (~gradGP*gradV).first()*gradGP/lGradGP/lGradGP;
    gradVT = gradVT/length(gradVT);

    arr X1, X0;
    for(uint i = 0; i < X.d0; i++) {
      if(Y(i) > 0.0) {
        X1.append(~X[i]);
      } else {
        X0.append(~X[i]);
      }
    }


    arr lg = zeros(2,3);
    lg[0] = pos;
    lg[1] = pos+gradGP/lGradGP;

    /*arr lgrad = zeros(2,3);
    arr gradientsOnSurfaceCopy = surfaceModelObject.get()().gradientsOnSurface;
    arr V = surfaceModelObject.get()().mesh.V;
    arr posGrad;
    if(gradientsOnSurfaceCopy.d0) {
      for(uint i = 0; i < gradientsOnSurfaceCopy.d0; i++) {
        arr actGrad = gradientsOnSurfaceCopy[i]/length(gradientsOnSurfaceCopy[i]);
        if(length(actGrad - gradReference.get()()) < 0.2) {
          posGrad = V[i];
          lgrad[0] = V[i];
          lgrad[1] = V[i] + gradReference.get()(); //+ actGrad/length(actGrad);
        }
      }
    }*/
    //arr newPos;
    arr l = zeros(2,3);
    //if(posGrad.N) {
    //  arr vG = posGrad - pos;
    //  arr s = vG - (~gradGP*vG).first()*gradGP/length(gradGP)/length(gradGP);
    //  s = s/length(s);
    //  newPos = pos + 0.1*s;
    //  l[0] = pos;
    //  l[1] = pos + s;
    //} else {
    //  newPos = pos + 0.1*gradVT;
    l[0] = pos;
    l[1] = pos + gradVT;
    //}


    /*arr x;
  x.setGrid(3,-0.4,0.9,10);
  arr grad = zeros(x.d0,3);
  for(uint i = 0; i < x.d0; i++) {
    arr g;
    gpCopy.gradient(g, x[i]);
    grad[i] = g; // /length(g);
  }*/
    arr path;
    //if(shortestPath.hasNewRevision()) {
    path = shortestPath.get()();
    //}

    ors::Vector n = ors::Vector(-gradGP/length(gradGP));
    arr V = ~n.generateOrthonormalSystemMatrix();

    arr ev1 = zeros(2,3);
    arr ev2 = zeros(2,3);
    arr ev3 = zeros(2,3);
    ev1[0] = pos;
    ev1[1] = pos+V[0];
    ev2[0] = pos;
    ev2[1] = pos+V[1];
    ev3[0] = pos;
    ev3[1] = pos+V[2];

    viewer->gl.lock.writeLock();
    plotClear();
    plotPoints(X0);
    plotPoints(X1);
    plotLine(l);
    plotLine(lg);
    if(path.d0 > 0) {
      plotLine(path);
    }
    plotLine(ev1);
    plotLine(ev2);
    plotLine(ev3);
    //plotLine(lgrad);
    //plotVectorField(x, grad);
    viewer->gl.lock.unlock();
  }
}



