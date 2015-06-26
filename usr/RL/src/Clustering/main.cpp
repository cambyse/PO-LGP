#include <QString>

#include <random>
#include <vector>
#include <tuple>
#include <map>
#include <chrono>

#include <lemon/list_graph.h>
#include <lemon/maps.h>

#include <ANN/ANN.h>

#include <util/util.h>
#include <util/graph_plotting.h>
#include <util/debug.h>

using namespace std;
using namespace lemon;
using util::Range;

int main(int argn, char ** args) {

    double scale = 10;
    const int cluster_n = 2;
    const int point_n = 100;

    // Generate Data
    map<pair<double,double>,pair<int,ListDigraph::Node>> data;
    {
        // construct a trivial random generator engine from a time-based seed:
        unsigned seed = chrono::system_clock::now().time_since_epoch().count();
        default_random_engine generator(seed);
        auto normal = [&](double mean, double stddev) -> double {
            normal_distribution<double> distribution(mean, stddev);
            return distribution(generator);
        };
        auto uni = [&](double min = 0, double max = 1) -> double {
            uniform_real_distribution<double> distribution(min, max);
            return distribution(generator);
        };
        for(int cluster_idx : Range(cluster_n)) {
            double x_mean = uni(-scale,scale);
            double y_mean = uni(-scale,scale);
            double x_std = uni(1e-1*scale,scale/2);
            double y_std = uni(1e-1*scale,scale/2);
            repeat(point_n) {
                data[make_pair(normal(x_mean, x_std), normal(y_mean, y_std))] = make_pair(cluster_idx, INVALID);
            }
        }
    }

    // Generate Graph via Nearest Neighbors
    ListDigraph         graph;                  // the graph structure
    ListDigraph::NodeMap<pair<double,double>> pos_map(graph); // position of nodes
    ListDigraph::NodeMap<int> class_map(graph); // class of nodes
    ListDigraph::ArcMap<double> dist_map(graph);// distance between nodes
    const int           neighbors_n = 5;       // number of NN to connect
    const int           dim = 2;                // dimension of input space
    ANNpointArray       dataPts;                // data points
    ANNpoint            queryPt;                // query point
    ANNidxArray         nnIdx;                  // near neighbor indices
    ANNdistArray        dists;                  // near neighbor distances
    ANNkd_tree*         kdTree;                 // search structure
    dataPts = annAllocPts(data.size(), dim);    // allocate data points
    queryPt = annAllocPt(dim);                  // allocate query point
    nnIdx = new ANNidx[neighbors_n];            // allocate near neigh indices
    dists = new ANNdist[neighbors_n];           // allocate near neighbor dists

    // write data to ANN structure
    int point_idx = 0;
    for(auto point : data) {
        dataPts[point_idx][0] = point.first.first;
        dataPts[point_idx][1] = point.first.second;
        ++point_idx;
    }

    // build search tree
    kdTree = new ANNkd_tree(
                dataPts,        // the data points
                data.size(),    // number of points
                dim             // dimension of space
                );

    // get neighbors for every ball an build graph
    for(auto & point : data) {

        // add node if not done in previous iterations
        if(point.second.second==INVALID) {
            point.second.second = graph.addNode();
        }
        ListDigraph::Node this_node = point.second.second;
        pos_map[this_node] = point.first;
        class_map[this_node] = point.second.first;

        // query point
        queryPt[0] = point.first.first;
        queryPt[1] = point.first.second;

        // search neighbors
        kdTree->annkSearch(
                    queryPt,       // query point
                    neighbors_n+1, // number of near neighbors
                    nnIdx,         // nearest neighbors (returned)
                    dists,         // distance (returned)
                    0              // error bound
                    );

        for(int idx : Range(1,neighbors_n)) {
            auto pos = make_pair(dataPts[nnIdx[idx]][0], dataPts[nnIdx[idx]][1]);
            auto & neighbor_class_and_node = data[pos];
            ListDigraph::Node neighbor_node = neighbor_class_and_node.second;
            if(neighbor_node==INVALID) {
                neighbor_node = graph.addNode();
                neighbor_class_and_node.second = neighbor_node;
                pos_map[neighbor_node] = pos;
                class_map[neighbor_node] = neighbor_class_and_node.first;
            }
            auto arc = graph.addArc(this_node, neighbor_node);
            dist_map[arc] = dists[idx];
        }
    }

    // print graph with clusters color-coded
    ListDigraph::NodeMap<QString> node_prop(graph);
    for(ListDigraph::NodeIt node(graph); node!=INVALID; ++node) {
        auto pos = pos_map[node];
        node_prop[node] = QString("pos=\"%1,%2!\" label=<%3> fillcolor=\"%4 1 0.9\"").
            arg(pos.first).
            arg(pos.second).
            arg(class_map[node]).
            arg((double)class_map[node]/cluster_n);
    }
    QString all_node_prop = QString("shape=circle style=filled truecolor=true fixedsize=true width=%1").arg(0.05*scale);
    util::plot_graph("graph.pdf", graph,
                     all_node_prop, &node_prop,
                     "", nullptr,
                     "",
                     true,
                     "neato");

    // discuss cluster :-)
    {
        // list of cluster id for every node
        ListDigraph::NodeMap<map<int,double>> cluster_lists(graph);
        ListDigraph::NodeMap<int> id_map(graph);
        // initialize with own id
        for(ListDigraph::NodeIt node(graph); node!=INVALID; ++node) {
            cluster_lists[node][graph.id(node)] = 1;
            id_map[node] = graph.id(node);
        }
        // try to convince neighbors
        while(true) {
            // wait before proceeding
            getchar();
            // update the cluster ids
            ListDigraph::NodeMap<map<int,double>> new_cluster_lists(graph);
            mapCopy(graph, cluster_lists, new_cluster_lists);
            for(ListDigraph::NodeIt node(graph); node!=INVALID; ++node) {
                // decide on a cluster id to argue for
                vector<int> ids;
                double max_score = -1;
                for(auto cluster : cluster_lists[node]) {
                    int id = cluster.first;
                    double score = cluster.second;
                    if(score>max_score) {
                        ids.clear();
                    }
                    if(score>=max_score) {
                        max_score = score;
                        ids.push_back(id);
                    }
                }
                DEBUG_EXPECT(0,ids.size()>0);
                int id = ids[rand()%ids.size()];
                id_map[node] = id;
                // talk to neighbors
                for(ListDigraph::OutArcIt arc(graph, node); arc!=INVALID; ++arc) {
                    new_cluster_lists[graph.target(arc)][id] += 1/dist_map[arc];
                }
                for(ListDigraph::InArcIt arc(graph, node); arc!=INVALID; ++arc) {
                    new_cluster_lists[graph.source(arc)][id] += 1/dist_map[arc];
                }
            }
            // copy back new map
            mapCopy(graph, new_cluster_lists, cluster_lists);
            // print graph (with old ids)
            {
                // get all ids and assign hue value
                map<int,double> id_to_hue;
                for(ListDigraph::NodeIt node(graph); node!=INVALID; ++node) {
                    id_to_hue[id_map[node]] = drand48();
                }
                // build node property map
                ListDigraph::NodeMap<QString> node_prop(graph);
                for(ListDigraph::NodeIt node(graph); node!=INVALID; ++node) {
                    auto pos = pos_map[node];
                    node_prop[node] = QString("pos=\"%1,%2!\" label=<%3> fillcolor=\"%4 1 0.9\"").
                        arg(pos.first).
                        arg(pos.second).
                        arg(class_map[node]).
                        arg(id_to_hue[id_map[node]]);
                }
                QString all_node_prop = QString("shape=circle style=filled truecolor=true fixedsize=true width=%1").arg(0.05*scale);
                util::plot_graph("graph.pdf", graph,
                                 all_node_prop, &node_prop,
                                 "", nullptr,
                                 true,
                                 "neato");
                DEBUG_OUT(0,id_to_hue.size() << " clusters");
            }
        }
    }

    return 0;
}
