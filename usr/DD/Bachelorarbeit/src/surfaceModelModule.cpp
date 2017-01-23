#include "surfaceModelModule.h"

#include <Control/TaskControllerModule.h>
#include <Control/RTControllerSimulation.h>
#include <Ors/orsviewer.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>
#include <Gui/color.h>
#include <Motion/motion.h>
#include "taskMapGPISP.h"

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

namespace {
  double getMeshUncertaintyMeasure(arr V, uintA T, arr variance) {
    //this is a surface integral of the variance over the surface, normalized by the area, see my bachelor thesis
    double A = 0.0;
    double S = 0.0;
    ors::Vector a,b,c;
    for(uint i = 0; i < T.d0; i++){
      a.set(V[T[i](1)]-V[T[i](0)]);
      b.set(V[T[i](2)]-V[T[i](0)]);
      c = a^b;
      A += c.length()/2.0;
      S += c.length()/2.0*(variance(T[i](0))+variance(T[i](1))+variance(T[i](2)))/3.0;
    }
    return S/A;
  }

  arr adjointOfThreeTimesThreeMatrix(const arr& A) {
    arr a = zeros(3,3);
    a(0,0) = A(1,1)*A(2,2)-A(1,2)*A(2,1);
    a(0,1) = -A(0,1)*A(2,2)+A(0,2)*A(2,1);
    a(0,2) = A(0,1)*A(1,2)-A(0,2)*A(1,1);

    a(1,0) = -A(1,0)*A(2,2)+A(1,2)*A(2,0);
    a(1,1) = A(0,0)*A(2,2)-A(0,2)*A(2,0);
    a(1,2) = -A(0,0)*A(1,2)+A(0,2)*A(1,0);

    a(2,0) = A(1,0)*A(2,1)-A(1,1)*A(2,0);
    a(2,1) = -A(0,0)*A(2,1)+A(0,1)*A(2,0);
    a(2,2) = A(0,0)*A(1,1)-A(0,1)*A(1,0);
    return a;
  }
}

SurfaceModelObject::SurfaceModelObject() {}

void SurfaceModelObject::calculateSurface(uint resolution) {
  CHECK(gp.X.d0, "gp has no data")
  ScalarFunction f = [this] (arr& ,arr& , const arr& X) {
    //double y, s;
    //gp.evaluate(X, y, s, false);
    return gp.evaluate(X);
    //return y;
  };
  mesh.setImplicitSurface(f, 0.3, 0.9, -0.4, 0.2, 0.4, 0.9, resolution);
}

void SurfaceModelObject::calculateVariance() {
  varianceOnSurface.clear();
  CHECK(mesh.V.d0, "Mesh not calculated")
  for(uint i = 0; i < mesh.V.d0; i++) {
    //double y, s;
    //gp.evaluate(mesh.V[i], y, s);
    double s = gp.evaluateVariance(mesh.V[i]);
    varianceOnSurface.append(s);
  }
}

void SurfaceModelObject::calculateGradientsOnSurface() {
  gradientsOnSurface.clear();
  CHECK(mesh.V.d0, "Mesh not calculated")
  for(uint i = 0; i < mesh.V.d0; i++) {
    arr grad;
    //gp.gradient(grad, mesh.V[i]);
    grad = gp.gradient(mesh.V[i]);
    gradientsOnSurface.append(~grad);
  }
}

void SurfaceModelObject::calculateVarianceGradientOnSurface() {
  varianceGradientsOnSurface.clear();
  CHECK(mesh.V.d0, "Mesh not calculated")
  for(uint i = 0; i < mesh.V.d0; i++) {
    arr grad;
    //gp.gradientV(grad, mesh.V[i]);
    //grad.reshapeFlat();
    grad = gp.gradientVariance(mesh.V[i]);
    varianceGradientsOnSurface.append(~grad);
  }
}

double SurfaceModelObject::calculateVarianceMeasure() const {
  CHECK(varianceOnSurface.d0 == mesh.V.d0 && mesh.V.d0, "Variance on surface not calculated")
  return getMeshUncertaintyMeasure(mesh.V, mesh.T, varianceOnSurface);
}

double SurfaceModelObject::calculateMeshDistance(const ors::Mesh& otherMesh) const {
  return ors::Mesh::meshMetric(otherMesh, mesh);
}

void SurfaceModelObject::plotVarianceOnSurface() {
  CHECK(varianceOnSurface.d0 == mesh.V.d0 && mesh.V.d0, "Variance on surface not calculated, varianceOnSurface.d0 = " << varianceOnSurface.d0 << ". mesh.V.d0 = " << mesh.V.d0)
  double minVar = varianceOnSurface.min();
  double maxVar = varianceOnSurface.max();
  //minVar = 0.1; //uncomment these if the color should be scaled the same from beginning to end
  //maxVar = 0.5;
  double a = (0.0-240.0)/(maxVar-minVar);
  double b = 240.0 - a*minVar;
  arr C = zeros(mesh.V.d0,3);
  for(uint i = 0; i < mesh.V.d0; i++) {
    mlr::Color color;
    color.setHsv(round(a*varianceOnSurface(i)+b)+1, 255, 255);
    C[i](0) = color.r;
    C[i](1) = color.g;
    C[i](2) = color.b;
  }
  mesh.C = C;
}

double SurfaceModelObject::calculateGaussianCurvature(const arr& pos) {
  arr grad = gp.gradient(pos);
  double lgradSqr = sumOfSqr(grad);
  arr H = gp.hessian(pos);
  return (~grad*adjointOfThreeTimesThreeMatrix(H)*grad).scalar()/lgradSqr/lgradSqr;
}

void SurfaceModelObject::calculateGaussianCurvatureOnSurface() {
  CHECK(mesh.V.d0, "Mesh not calculated")
  for(uint i = 0; i < mesh.V.d0; i++) {
    gaussianCurvatureOnSurface.append(calculateGaussianCurvature(mesh.V[i]));
  }
}

void SurfaceModelObject::plotGaussianCurvatureOnSurface() {
  CHECK(gaussianCurvatureOnSurface.d0 == mesh.V.d0 && mesh.V.d0, "Curvature on surface not calculated")
  double minVar = gaussianCurvatureOnSurface.min();
  double maxVar = gaussianCurvatureOnSurface.max();
  //minVar = 0.1; //uncomment these if the color should be scaled the same from beginning to end
  //maxVar = 0.5;
  double a = (0.0-240.0)/(maxVar-minVar);
  double b = 240.0 - a*minVar;
  arr C = zeros(mesh.V.d0,3);
  for(uint i = 0; i < mesh.V.d0; i++) {
    mlr::Color color;
    color.setHsv(round(a*gaussianCurvatureOnSurface(i)+b)+1, 255, 255);
    C[i](0) = color.r;
    C[i](1) = color.g;
    C[i](2) = color.b;
  }
  mesh.C = C;
}

arr SurfaceModelObject::computeGeodesicEuklideanPathOnSurface(const arr& startPos, const arr& targetPos, arr& startOnSurface, arr& targetOnSurface) {
  return computeGeodesicEuklideanPathOnSurface(startPos, targetPos, this->mesh, startOnSurface, targetOnSurface);
}

arr SurfaceModelObject::computeGeodesicPathOnSurface(const arr& startPos, const arr& targetPos, ors::Mesh mesh, std::function<double(const arr&, const arr&, uint, uint)> distanceFunction, arr& startOnSurface, arr& targetOnSurface) {
  //calculate the vertex indices that have the shortest distances to startPos and targetPos, respectively. O(|V|)
  uint startIndex = 0, targetIndex = 0;
  for(uint i = 0; i < mesh.V.d0; i++) {
    if(length(mesh.V[i]-startPos) < length(mesh.V[startIndex]-startPos)) {
      startIndex = i;
    }
    if(length(mesh.V[i]-targetPos) < length(mesh.V[targetIndex]-targetPos)) {
      targetIndex = i;
    }
  }
  if(&startOnSurface) startOnSurface = mesh.V[startIndex];
  if(&targetOnSurface) targetOnSurface = mesh.V[targetIndex];

  typedef std::pair<uint, uint> Edge;
  typedef std::set<Edge> Edges;
  typedef boost::adjacency_list <boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, double> > DijkstraGraph;
  typedef boost::graph_traits<DijkstraGraph>::vertex_descriptor Node;

  Edges edges; //here he edges with distances are stored
  DijkstraGraph G(mesh.V.d0); //the whole dijkstra graph

  //translating the mesh into an graph, O(|T|*log(|T|)) complexity
  for(uint i = 0; i < mesh.T.d0; i++) {
    mlr::Array<uint> T = mesh.T[i]; // the actual triangle
    for(uint j = 0; j <= 2; j++) {
      Edge e(T(j),T((j+1)%3));
      //test if edge is already in the graph, as mesh contains many edges multiple times
      if(edges.find(e) == edges.end()) {
        double distance = distanceFunction(mesh.V[e.first], mesh.V[e.second], e.first, e.second);
        edges.insert(e);
        boost::add_edge(e.first, e.second, distance, G);
      }
    }
  }

  std::vector<Node> predecessors(mesh.V.d0); //here the predecessors are stored, starting from the start node

  // run dijkstra. O(|V|*log(|V|) + |E|)
  boost::dijkstra_shortest_paths(G, startIndex, boost::predecessor_map(&predecessors[0]));

  //extract path from start to target.
  arr path;
  path.append(~mesh.V[targetIndex]);
  uint currentNodeIndex = targetIndex;
  while(true) {
    uint pre = predecessors[currentNodeIndex];
    path.append(~mesh.V[pre]);
    if(pre == startIndex) break;
    currentNodeIndex = pre;
  }
  path.reverseRows();
  return path;
}

arr SurfaceModelObject::computeGeodesicEuklideanPathOnSurface(const arr& startPos, const arr& targetPos, ors::Mesh mesh, arr& startOnSurface, arr& targetOnSurface) {
  auto eukledianDistance = [](const arr& pos1, const arr& pos2, uint, uint)->double {
    return length(pos1-pos2);
  };
  return computeGeodesicPathOnSurface(startPos, targetPos, mesh, eukledianDistance, startOnSurface, targetOnSurface);
}

arr SurfaceModelObject::computeGeodesicVariancePathOnSurface(const arr& startPos, const arr& targetPos, ors::Mesh mesh, const arr& varianceSurface, arr& startOnSurface, arr& targetOnSurface) {
  auto varianceDistance = [&varianceSurface] (const arr& pos1, const arr& pos2, uint pos1Index, uint pos2Index)->double {
    return 1.0/(length(pos1-pos2)*(varianceSurface(pos1Index)+varianceSurface(pos2Index))/2.0);
  };
  return computeGeodesicPathOnSurface(startPos, targetPos, mesh, varianceDistance, startOnSurface, targetOnSurface);
}

arr SurfaceModelObject::smoothGeodesicPathWithKOMO(const arr& dijkstraPath) {
  ors::KinematicWorld world(mlr::mlrPath("../usr/DD/Bachelorarbeit/src/3DRobot.ors"));
  world.setJointState(dijkstraPath[0]);
  MotionProblem MP(world, false);
  //MP.k_order = 1;
  MP.T = dijkstraPath.d0;

  Task *t;
  t = MP.addTask("geodesicDistance", new TaskMap_Transition(MP.world), sumOfSqrTT);
  t->map.order=1;
  t->setCostSpecs(0, MP.T, {0.}, 1.0);

  t = MP.addTask("onSurface", new TaskMap_GPISP(this->gp, MP.world, "endeffR"), sumOfSqrTT);
  t->setCostSpecs(0, MP.T, {0.0}, 1000.0);

  t = MP.addTask("start", new TaskMap_Default(posTMT, MP.world, "endeffR"), sumOfSqrTT);
  t->setCostSpecs(0,0, dijkstraPath[0], 10.0);

  t = MP.addTask("target", new TaskMap_Default(posTMT, MP.world, "endeffR"), sumOfSqrTT);
  t->setCostSpecs(MP.T-2,MP.T, dijkstraPath[dijkstraPath.d0-1], 10.0);

  arr komoPath = dijkstraPath;

  optNewton(komoPath , Convert(MP), OPT(verbose=true));
  komoPath.reshape(MP.T, komoPath.N/MP.T);
  Graph result = MP.getReport(true);
  return komoPath;
}

//=====================================================================


SurfaceModelModule::SurfaceModelModule(Roopi& roopi)
  : Thread("surfaceModel", 0.2)
  , roopi(roopi)
  , gpSurface(this, "gpSurface")
  , log("renameMeGPData") {}

SurfaceModelModule::~SurfaceModelModule() {
  threadClose();
}

void SurfaceModelModule::open() {
  OrsPoseViewer* viewer = getThread<OrsPoseViewer>("OrsPoseViewer");
  viewer->gl.add(glDrawPlot, &plotModule);

  gpSurface.writeAccess();
  //GaussianProcessGaussKernel* kernel = new GaussianProcessGaussKernel(1.0, 0.2);
  GaussianProcessKernel* kernel = new GaussianProcessInverseMultiQuadricKernel(0.2); //Test 0.3 with 0.01 COOL
  gpSurface().m = 1.0;
  gpSurface().obsVar = 0.3;
  gpSurface().setKernel(kernel);
  gpSurface.deAccess();
  posMap = new TaskMap_Default(posTMT, roopi.tcm()->modelWorld.get()(), "endeffR");
}

void SurfaceModelModule::step() {
  arr fR = roopi.getFTRight();
  arr pos;
  posMap->phi(pos, NoArr, roopi.tcm()->modelWorld.get()());
  X.append(~pos);
  log.write("X", ~pos);
  if(fR(2) < -2.0) {
    Y.append(ARR(0.0));
    log.write("Y", ARR(0.0));
  } else {
    Y.append(ARR(1.0));
    log.write("Y", ARR(1.0));
  }

  gpSurface.writeAccess();
  //gpSurface().X = X;
  //gpSurface().Y = Y;
  //gpSurface().recompute();
  gpSurface().appendObsRecompute(pos,Y.last()); //TODO move this above
  gpSurface.deAccess();

  cout << "#Data points: " <<  X.d0 << endl;
}

void SurfaceModelModule::close() {
  delete posMap;
}
