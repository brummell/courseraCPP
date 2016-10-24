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
#include <fstream>
//THIS WILL FIND USE IN PGMS AS WELL AS HPC COMPUTATION GRAPH CALCULATIONS

using namespace std;


class Graph {
    double INF = numeric_limits<double>::infinity();
    int QNAN = numeric_limits<int>::quiet_NaN();
    using min_edge = pair<pair<int, int>, double>;

    struct Vertex {
        int id;
//        unordered_map<int, double> edges{{id, 0.0}}; // TODO: DROP THE AUTO PART MAYBE?
        unordered_map<int, double> edges;

    };

//    struct Edge {
//        // TODO: sort node identifiers (indices) and make in lowest to establish convention to ease reading output
//        pair<int, int> nodes;
//        double weight;
//
//        // comparator for Edge class, returns minimum based on weight, but will sort on edge node for STL sake in case of tie
//        auto comparator = [](const Edge &left, const Edge &right) { // lift this into higher namespace and reuse accross stl stuff
//            if (left.second != right.second) { return left.second < right.second; }
//            else if (left.first.first != right.first.first) { return left.first.first < right.first.first; }
//            else { return left.first.second < right.first.second; }
//        };
//
//    };

    private:
        unordered_map<int, int> index_map; //template for when node names aren't just the integers of where they were placed
        vector<Vertex> graph;
        unordered_map<int, int> vertex_map;
        pair<int, int> edge_range;
        double target_density;
        int size; // TODO: change name
        string filename;
        random_device rand;

    public:
        // constructors
        Graph() : graph() {};

        // creates graph only of known size
        Graph(int size) : size(size), graph() {
            for (int i = 0; i < size; ++i) { graph.push_back(Vertex{i}); }
        };

        // creates graph via filename pointing to file in the format:
        // line 1: size_of_graph
        // line 2: vertex_1 vertex_2 edge_weight
        //  ...
        Graph(string filename) : filename(filename), graph() {
            ifstream infile(filename);
            infile >> size;
            cout << size << endl;
            for (int i = 0; i < size; ++i) { graph.push_back(Vertex{i}); }
            int vertex_a, vertex_b, edge_weight;
            while (infile >> vertex_a >> vertex_b >> edge_weight) {
                // we should not trust that graph input format is perfectly symmetrical, hence the testing
                if (not graph[vertex_a].edges.count(vertex_b)) {
                        graph[vertex_a].edges[vertex_b] = edge_weight;
                        graph[vertex_b].edges[vertex_a] = edge_weight;
                    }
            }
        }

        // create random graph with given size, edge density, and edge weight range
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
                // we should not trust that graph input format is perfectly symmetrical, hence the testing
                if (vertex_a != vertex_b and not graph[vertex_a].edges.count(vertex_b)) {
                    double weight = weight_dstr(gen);
                    graph[vertex_a].edges[vertex_b] = weight;
                    graph[vertex_b].edges[vertex_a] = weight;
                    edge_count += 1;
                }
            }
        }


    friend auto find_minimum_spanning_tree(const Graph &graph);

    friend auto run_mc_prims_mst(const Graph &graph);

    friend ostream &operator<<(ostream &os, const Graph &graph);

};


// use Prim's algorithm to find a minimum spanning tree, here using a naive implementation
// as per the instructions, we are assuming a connected UAG, with non-negative weights
auto find_minimum_spanning_tree(const Graph &graph) {
    // an MST of a UAG must be of the same size, it's edge count would be known as well (V-1), but that is of no use now.
    Graph mst{graph.size};

    // remember, no node gets touched twice
    // TODO: RIGHT NOW ASSUMING CONNECTED COMPONENT
    // TODO: TIE BREAKING EDGE WEIGHTS?


    // TODO: FOR HW, WRITE EXPLANATION ABOUT SET ABSTRACTION
    set<int> VminX, X, V;
    vector<double> A(graph.size, graph.INF);
    vector<vector<int>> B(graph.size);
    for (int i = 0; i < graph.size; ++i) {
        V.insert(i); // This can be optimed away probably
        VminX.insert(i);
    }

    VminX.erase(0);
    X.insert(0);
    A[0] = 0;
    B[0].push_back(0);
    while (VminX.size() != 0) {
        Graph::min_edge mini_edge{{graph.QNAN, graph.QNAN}, graph.INF};
        for (auto x : X) {
            cout << "working with vertex:  " << x << endl;
            for (auto edge : graph.graph[x].edges) {
                if (VminX.count(edge.first) and edge.second < mini_edge.second) {
                    mini_edge.first.first = x;
                    mini_edge.first.second = edge.first;
                    mini_edge.second = edge.second;
                    cout << "Found min: " << "(" << mini_edge.first.first << ", " << mini_edge.first.second << "), " << mini_edge.second << endl;
                }

            }
            cout << "Found true min for vertex " << x << ": " << "(" << mini_edge.first.first << ", " << mini_edge.first.second << "), " << mini_edge.second << endl;
        }
        cout << "next cycle!" << endl << endl;
        VminX.erase(mini_edge.first.second);
        X.insert(mini_edge.first.second);
        A[mini_edge.first.second] = mini_edge.second;
        B[mini_edge.first.first].push_back(mini_edge.first.second);
        // TODO: this whole thing could go away if we had Graph::edges

        // TODO: change Graph.graph to indices I think or vertices
        mst.graph[mini_edge.first.first].edges[mini_edge.first.second] = mini_edge.second;
        mst.graph[mini_edge.first.second].edges[mini_edge.first.first] = mini_edge.second;
    }
    cout << mst << endl;
    return pair<decltype(A), decltype(B)> {A,B};
};

auto run_mc_prims_mst(const Graph &graph) {
    cout << "running graph with size: " << graph.size << endl;

    auto results = find_minimum_spanning_tree(graph);
    for (int i = 0; i < graph.size; i++) {
        cout << "MST for generated graph " << i << ": ";
        for (const auto &v : results.second[i]) {
            cout << v << " -> ";
        }
        cout << right << " (" << results.first[i] << ")" << endl;
    }
    cout << "MST total cost: "
         << accumulate(begin(results.first) + 1, end(results.first), 0)<< endl;
};


ostream &operator<<(ostream &os, const Graph &g) {
    for (int i{0}; i < g.graph.size(); ++i) {
        os << i << " -> ";
        for (auto &edge : g.graph[i].edges) {
            os << "{" << edge.first << ": " << edge.second << "}, ";
        }
        os << endl;
    }
    return os;
}


int main() {
    string filename{"/Users/dbrummell13/ClionProjects/Coursera/SampleTestData_mst_data.txt"};
    Graph graph(filename);
    cout << setprecision(5) << graph << endl;
    run_mc_prims_mst(graph);

    return 0;
}
