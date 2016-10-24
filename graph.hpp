#include <random>
#include <vector>
#include <unordered_map>
#include <set>
#include <cassert>
#include <unordered_set>
#include <iomanip>
#include <string>
#include <queue>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
//THIS WILL FIND USE IN PGMS AS WELL AS HPC COMPUTATION GRAPH CALCULATIONS

using namespace std;


class Graph {
    double INF = numeric_limits<double>::infinity();
    int QNAN = numeric_limits<int>::quiet_NaN();
    using min_edge = pair<pair<int, int>, double>;

    struct Vertex {
        int id;
        unordered_map<int, double> edges{{id, 0.0}};
    };

    struct Edge {
        int in;
        int out;
        double weight;
    };

private:
    unordered_map<int, int> index_map; //template for when node names aren't just the integers of where they were placed
    vector<Vertex> graph;
    unordered_map<int, int> vertex_map;
    pair<int, int> edge_range;
    double target_density;
    int size; // TODO: change name
    // Random seed
    random_device rand;

public:
    // constructors
    Graph() : graph() {};

    // create random graph
    Graph(double target_density, int size, pair<int, int> edge_range) : target_density(
            target_density), edge_range(edge_range), size(size), graph() {
        for (int i = 0; i < size; ++i) { graph.push_back(Vertex{i}); }
        // Init Mersenne Twister PRNG
        mt19937 gen(rand());
        uniform_int_distribution<> edge_dstr(0, size - 1);
        uniform_real_distribution<> weight_dstr(edge_range.first, edge_range.second);
        int max_possible_edges = (size * (size - 1)) / 2;
        int necessary_edges = floor(max_possible_edges * target_density);
        int edge_count{0};
        while (edge_count <= necessary_edges) {  // Generate connections
            int vertex_a = edge_dstr(gen);
            int vertex_b = edge_dstr(gen);
            if (vertex_a != vertex_b and not graph[vertex_a].edges.count(vertex_b)) {
                double weight = weight_dstr(gen);
                graph[vertex_a].edges[vertex_b] = weight;
                graph[vertex_b].edges[vertex_a] = weight;
                edge_count += 1;
            }
        }
    };

    auto V() const {
        // returns set of vertices in the graph, by index (or name?)
        set<int> V;
        for (int i = 0; i < graph.size(); i++) {
            V.insert(i);
        }
        return V;
    };

//        E (G): returns the number of edges in the graph

    auto edges(const int &v) { return 0 }; // set of all adjacent nodes

    auto neighbors(const int &v) { // lists set of all adjacent nodes
        set<int> U;
        for (const auto &edge : graph[v].edges) {
            U.insert(edge.first);
        }
        return U;
    };

    auto is_adjacent_to(const int &u, const int &v) { // tests whether there is an edge from node x to node y.
        return graph[u].edges.count(v) != 0;
    };

    // template <typename T>
    void insert_vertex(
            const int &Vert) { // adds new vertex, with new type, if it is not there. adds vert to map, pushes back new index/vertex
        Vertex v{};
        graph.push_back(v);
    };

    void delete_vertex(const int &u) { // removes the edge from x to y, if it is there.

    };

    void insert_edge(const int &u, const int &v,
                     const double &weight) { // adds edge from u to v, if it is not there, updates it otherwise
        graph[u].edges[v] = weight; // TODO: should take an edge, maybe against operator[]
        graph[v].edges[u] = weight; // TODO: return index?
    };

    void delete_edge(const int &u, const int &v) { // removes the edge from u to v, if it is there.
        graph[u].edges.erase(v);
        graph[v].edges.erase(u);
    };


//        get_node_value (G, x): returns the value associated with the node x.
//        set_node_value( G, x, a): sets the value associated with the node x to a.
//        get_edge_value( G, x, y): returns the value associated to the edge (x,y).
//        set_edge_value (G, x, y, v): sets the value associated to the edge (x,y) to v.
    void reindex() {
        //after a certain number of deletes, compress the vector and reindex the nodes and edges
    };

    friend ostream &operator<<(ostream &os, const Graph &graph);

};


ostream &operator<<(ostream &os, const Graph &g) {
    for (int i{0}; i < g.graph.size(); ++i) {
        os << i << " -> ";
        for (auto &edge : g.graph[i].edges) {
            os << "{" << edge.first << ", " << edge.second << "}, ";
        }
        os << endl;
    }
    return os;
}