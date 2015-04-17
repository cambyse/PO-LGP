//=======================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// This file is part of the Boost Graph Library
//
// You should have received a copy of the License Agreement for the
// Boost Graph Library along with the software; see the file LICENSE.
// If not, contact Office of Research, University of Notre Dame, Notre
// Dame, IN 46556.
//
// Permission to modify the code and to distribute modified code is
// granted, provided the text of this NOTICE is retained, a notice that
// the code was modified is included with the above COPYRIGHT NOTICE and
// with the COPYRIGHT NOTICE in the LICENSE file, and that the LICENSE
// file is distributed with the modified code.
//
// LICENSOR MAKES NO REPRESENTATIONS OR WARRANTIES, EXPRESS OR IMPLIED.
// By way of example, but not limitation, Licensor MAKES NO
// REPRESENTATIONS OR WARRANTIES OF MERCHANTABILITY OR FITNESS FOR ANY
// PARTICULAR PURPOSE OR THAT THE USE OF THE LICENSED SOFTWARE COMPONENTS
// OR DOCUMENTATION WILL NOT INFRINGE ANY PATENTS, COPYRIGHTS, TRADEMARKS
// OR OTHER RIGHTS.
//=======================================================================

#if 1
#include <boost/config.hpp>
#include <iostream>
#include <vector>
#include <utility>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
using namespace boost;
using namespace std;

//typedef adjacency_list<vecS, vecS, bidirectionalS, VertexProperty, edge_p> Graph;
typedef struct vert{
   std::string name;
 };

 typedef struct edgepp{
   int capacity;
   int weight;
 };



typedef adjacency_list<listS, vecS, bidirectionalS, vert, edgepp> Graph;
//Graph g;

//typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;



template <class WeightMap,class CapacityMap>
class edge_writer {
public:
  edge_writer(WeightMap w, CapacityMap c) : wm(w),cm(c) {}
  template <class Edge>
  void operator()(ostream &out, const Edge& e) const {
    out << "[label=\"" << wm[e] << "\", taillabel=\"" << cm[e] << "\"]";
  }
private:
  WeightMap wm;
  CapacityMap cm;
};

template <class WeightMap, class CapacityMap>
inline edge_writer<WeightMap,CapacityMap>
make_edge_writer(WeightMap w,CapacityMap c) {
  return edge_writer<WeightMap,CapacityMap>(w,c);
}





template <class Graph>
void print(Graph& g) {
  typename Graph::vertex_iterator i, end;
  typename Graph::out_edge_iterator ei, edge_end;
  for(boost::tie(i,end) = vertices(g); i != end; ++i) {
    cout << *i << " --> ";
    for (boost::tie(ei,edge_end) = out_edges(*i, g); ei != edge_end; ++ei)
      cout << target(*ei, g) << "  ";
    cout << endl;
  }
}
std::size_t myrand(std::size_t N) {
  std::size_t ret = rand() % N;
  //  cout << "N = " << N << "  rand = " << ret << endl;
  return ret;
}



template <class Graph>
bool check_edge(Graph& g, std::size_t a, std::size_t b) {
  typedef typename Graph::vertex_descriptor Vertex;
  typename Graph::adjacency_iterator vi, viend, found;
  boost::tie(vi, viend) = adjacent_vertices(vertex(a,g), g);
  found = find(vi, viend, vertex(b, g));
  if ( found == viend )
    return false;
  return true;
}



int main(int, char*[])
{
  std::size_t N = 5;
  Graph g(N);
  int i;
  bool is_failed = false;
  for (i=0; i<6; ++i) {
    std::size_t a = myrand(N), b = myrand(N);
    while ( a == b ) b = myrand(N);
    cout << "edge edge (" << a << "," << b <<")" << endl;
    //add edges
    //EdgeProperty e = myrand(3);
    edgepp prop;
    prop.weight = 5;
    prop.capacity = 4;
    add_edge(a, b, prop, g);


    is_failed =  is_failed || (! check_edge(g, a, b) );
  }

  if ( is_failed )
    cerr << "    Failed."<< endl;
  else
    cerr << "           Passed."<< endl;

  print(g);

  // write the dot file

 std::ofstream dotfile ("abc.dot");
 write_graphviz (dotfile, g,
                 //boost::default_writer(),
                 boost::make_label_writer("abcdefgh"),
                 make_edge_writer(boost::get(&edgepp::weight,g),boost::get(&edgepp::capacity,g)));

  return 0;
}


#else

#include <boost/graph/graphviz.hpp>
#include<vector>
using namespace std;

enum files_e { dax_h, yow_h, boz_h, zow_h, foo_cpp,
               foo_o, bar_cpp, bar_o, libfoobar_a,
               zig_cpp, zig_o, zag_cpp, zag_o,
                 libzigzag_a, killerapp, N };
const char* name[] = { "dax.h", "yow.h", "boz.h", "zow.h", "foo.cpp",
                       "foo.o", "bar.cpp", "bar.o", "libfoobar.a",
                       "zig.cpp", "zig.o", "zag.cpp", "zag.o",
                       "libzigzag.a", "killerapp" };

int main(int,char*[])
{

  typedef pair<int,int> Edge;
  Edge used_by[] = {
    Edge(dax_h, foo_cpp), Edge(dax_h, bar_cpp), Edge(dax_h, yow_h),
    Edge(yow_h, bar_cpp), Edge(yow_h, zag_cpp),
    Edge(boz_h, bar_cpp), Edge(boz_h, zig_cpp), Edge(boz_h, zag_cpp),
    Edge(zow_h, foo_cpp),
    Edge(foo_cpp, foo_o),
    Edge(foo_o, libfoobar_a),
    Edge(bar_cpp, bar_o),
    Edge(bar_o, libfoobar_a),
    Edge(libfoobar_a, libzigzag_a),
    Edge(zig_cpp, zig_o),
    Edge(zig_o, libzigzag_a),
    Edge(zag_cpp, zag_o),
    Edge(zag_o, libzigzag_a),
    Edge(libzigzag_a, killerapp)
  };
  const int nedges = sizeof(used_by)/sizeof(Edge);
  int weights[nedges];
  std::fill(weights, weights + nedges, 1);

  using namespace boost;

  typedef adjacency_list< vecS, vecS, directedS,
      property< vertex_color_t, default_color_type >,
      property< edge_weight_t, int >
    > Graph;
  Graph g(used_by, used_by + nedges, weights, N);

  std::ofstream dotfile ("abc.dot");
  write_graphviz(dotfile, g, make_label_writer(name));
}

#endif
