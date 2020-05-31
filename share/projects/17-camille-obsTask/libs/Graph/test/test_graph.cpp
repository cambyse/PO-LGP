#include <KOMO/komo.h>

#include <ngraph.h>

#include <gtest/gtest.h>
#include <dlib/clustering.h>

using namespace std;

namespace NGraph
{
static void to_gv(const Graph & G, std::ostream & os)
{
  os << "graph G{\n";

  for (Graph::const_iterator p = G.begin(); p != G.end(); p++)
  {
      if (Graph::out_neighbors(p).size() == 0  &&
          Graph::in_neighbors(p).size() == 0)
      {
        os << Graph::node(p) << ";\n";
      }
  }
  for (Graph::const_iterator p = G.begin(); p != G.end(); p++)
  {
      const Graph::vertex_set &out = Graph::out_neighbors(p);
      Graph::vertex from = Graph::node(p);
      for (Graph::vertex_set::const_iterator q = out.begin();
                  q != out.end(); q++)
      {
          //if (from <= *q)
          os << from << "--" << *q << " ;\n";
      }
  }

  os << "}";
}

static void to_file(const Graph & G, const std::string & filepath, bool png=false)
{
  std::ofstream file;
  file.open(filepath, ios::out);

  to_gv(G, file);

  file.close();

  if(png)
  {
    std::string name_copy(filepath);
    const std::string ext( ".gv" );
    std::string new_name = name_copy.replace( name_copy.find( ext ), ext.length(), ".png" );

    std::stringstream ss;
    ss << "dot"   << " ";
    ss << "-Tpng" << " ";
    ss << "-o"    << " ";
    ss << new_name << " ";
    ss << filepath;

    system( ss.str().c_str() );
  }
}

static Graph from_hessian(const arr& H)
{
  NGraph::Graph G;

  for(auto i = 0; i < H.d0; ++i)
  {
    for(auto j = 0; j < i; ++j)
    {
      if(H(i, j))
      {
        G.insert_edge(i, j);

        std::cout << i << " " << j << std::endl;
      }
    }
  }

  return G;
}

}

static arr build_simple_decoupled()
{
  arr J = zeros(3, 4);
  J(0, 0) = 1;
  J(0, 2) = 1;
  J(1, 1) = 1;
  J(1, 3) = 1;
  J(2, 1) = 1;
  J(2, 3) = 1;

  //std::cout << J << std::endl;
  auto H = comp_At_A(J);

  return H;
}

static arr build_simple_light_coupling()
{
  arr J = zeros(5, 6);
  J(0, 0) = 1; J(0, 2) = 1;
  J(1, 2) = 1; J(1, 4) = 1;
  J(2, 1) = 1; J(2, 3) = 1;
  J(3, 3) = 1; J(3, 5) = 1;
  J(4, 2) = 1; J(4, 3) = 1;

  //std::cout << J << std::endl;
  auto H = comp_At_A(J);

  return H;
}


static arr build_medium_light_coupling()
{
  arr J = zeros(5, 12);
  J(0, 0) = 1;  J(0, 1) = 1; J(0, 4) = 1; J(0, 5) = 1;
  J(1, 4) = 1;  J(1, 5) = 1; J(1, 8) = 1; J(1, 9) = 1;
  J(2, 2) = 1;  J(2, 3) = 1; J(2, 6) = 1; J(2, 7) = 1;
  J(3, 6) = 1;  J(3, 7) = 1; J(3, 10) = 1; J(3, 11) = 1;
  J(4, 5) = 1;  J(4, 6) = 1;

  std::cout << J << std::endl;
  auto H = comp_At_A(J);

  return H;
}


TEST(Graph, InsertEdge)
{
  NGraph::Graph A;
  A.insert_edge(3,4);
  A.insert_edge(4,5);

  EXPECT_EQ(3, A.num_vertices());
  EXPECT_EQ(2, A.num_edges());
}

TEST(Graph, SaveToFile)
{
  NGraph::Graph B;
  B.insert_edge(5,2);
  B.insert_edge(3,4);
  B.insert_edge(4,5);

  NGraph::to_gv(B, std::cout);
  NGraph::to_file(B, "graph.gv", true);
}

TEST(Graph, RelationJacobianHessian)
{
  auto H = build_simple_decoupled();

  EXPECT_EQ(H.d0, 4);
}

TEST(Graph, BuildGraphOutOfHessianDecoupled)
{
  auto H = build_simple_decoupled();

  NGraph::Graph G = NGraph::from_hessian(H);

  NGraph::to_file(G, "simple_decoupled.gv", true);
}

TEST(Graph, BuildGraphOutOfHessianLightCoupling)
{
  auto H = build_simple_light_coupling();

  NGraph::Graph G = NGraph::from_hessian(H);

  NGraph::to_file(G, "build_simple_light_coupling.gv", true);
}

TEST(Graph, BuildGraphOutOfHessianMediumLightCoupling)
{
  auto H = build_medium_light_coupling();

  NGraph::Graph G = NGraph::from_hessian(H);

  NGraph::to_file(G, "build_medium_light_coupling.gv", true);
}


std::vector<unsigned long> spectral_cluster (
    const dlib::matrix<double>& A,
    const unsigned long num_clusters
)
{
    using namespace dlib;

    DLIB_CASSERT(num_clusters > 1,
        "\t std::vector<unsigned long> spectral_cluster(k,samples,num_clusters)"
        << "\n\t num_clusters can't be 0."
        );

//    // compute the similarity matrix.
//    matrix<double> K(samples.size(), samples.size());
//    for (long r = 0; r < K.nr(); ++r)
//        for (long c = r+1; c < K.nc(); ++c)
//            K(r,c) = K(c,r) = (double)k(samples[r], samples[c]);
//    for (long r = 0; r < K.nr(); ++r)
//        K(r,r) = 0;

    auto K = A; // copy

    matrix<double,0,1> D(K.nr());
    for (long r = 0; r < K.nr(); ++r)
        D(r) = sum(rowm(K,r));
    D = sqrt(reciprocal(D));
    K = diagm(D)*K*diagm(D);
    matrix<double> u,w,v;
    // Use the normal SVD routine unless the matrix is really big, then use the fast
    // approximate version.
    if (K.nr() < 1000)
        svd3(K,u,w,v);
    else
        svd_fast(K,u,w,v, num_clusters+100, 5);
    // Pick out the eigenvectors associated with the largest eigenvalues.
    rsort_columns(v,w);
    v = colm(v, range(0,num_clusters-1));
    // Now build the normalized spectral vectors, one for each input vector.
    std::vector<matrix<double,0,1> > spec_samps, centers;
    for (long r = 0; r < v.nr(); ++r)
    {
        spec_samps.push_back(trans(rowm(v,r)));
        const double len = length(spec_samps.back());
        if (len != 0)
            spec_samps.back() /= len;
    }
    // Finally do the K-means clustering
    pick_initial_centers(num_clusters, centers, spec_samps);
    find_clusters_using_kmeans(spec_samps, centers);
    // And then compute the cluster assignments based on the output of K-means.
    std::vector<unsigned long> assignments;
    for (unsigned long i = 0; i < spec_samps.size(); ++i)
        assignments.push_back(nearest_center(centers, spec_samps[i]));

    return assignments;
}


TEST(Graph, UseDLib)
{
  using namespace dlib;

  auto H = build_medium_light_coupling();

  auto add_edge = [](uint from, uint to, matrix<double> & A)
  {
    A(from, to) = A(to, from) = 1.0;
  };

  matrix<double> A(12, 12);

  for(auto i = 0; i < A.nr(); ++i)
    for(auto j = 0; j < A.nc(); ++j)
      A(i, j) = 0.0;

  add_edge(1, 0, A);
  add_edge(3, 2, A);
  add_edge(4, 0, A);
  add_edge(4, 1, A);
  add_edge(5, 0, A);
  add_edge(5, 1, A);
  add_edge(5, 4, A);
  add_edge(6, 2, A);
  add_edge(6, 3, A);
  add_edge(6, 5, A);
  add_edge(7, 2, A);
  add_edge(7, 3, A);
  add_edge(7, 6, A);
  add_edge(8, 4, A);
  add_edge(8, 5, A);
  add_edge(9, 4, A);
  add_edge(9, 5, A);
  add_edge(9, 8, A);
  add_edge(10, 6, A);
  add_edge(10, 7, A);
  add_edge(11, 6, A);
  add_edge(11, 7, A);
  add_edge(11, 10, A);
  add_edge(11, 12, A);

  std::cout << A << std::endl;

  // Finally, we can also solve the same kind of non-linear clustering problem with
  // spectral_cluster().  The output is a vector that indicates which cluster each sample
  // belongs to.  Just like with kkmeans, it assigns each point to the correct cluster.
  std::vector<unsigned long> assignments = spectral_cluster(A, 2);
  cout << mat(assignments) << endl;
}



////////////////////////////////
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}

