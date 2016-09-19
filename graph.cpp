//
// Created by Brummell, Doug on 8/14/16.
//

// undirected acyclic graph, strictly positive weights (0==no edge)
// adjacency list representation for now... binary heap allows faster Dykstra's
// dumb index version, should use other branch in git with pointer one some day... getter/setters to abstract over both
// WILL CREATE DIFFERENT  STRUCTS ETC DEPENDING ON CONSTRUCTION (DENSITY, DIRECTEDNESS, ETC)
// TODO: Add DAG, UAG, DCG, UCG options, unweighted as well
// TODO: add iterator?

//struct Edge {
//    int vertex;
//    double weight; // comparable, for sake of < op for set insertion
//};
//inline bool operator<(const Edge &lhs, const Edge &rhs) {
//    return lhs.weight < rhs.weight;
//}

//#include "graph.h"
#include <random>
#include <vector>
#include <unordered_map>
#include <set>
#include <cassert>
#include <unordered_set>
#include <iomanip>
#include <string>
#include <iostream>
#include <cmath>


using namespace std;




struct Vertex {
    // possible implement a conversion to int type, possible through name map, to pass vertex more easily to functions
    int id;
    unordered_map<int, double> edges; // TODO: should be unordered, once reasonable has defined
};

//template <typename T>
class Graph {
//    using iset = unordered_set<int>;
// DESIGN PHILOSOPHY CONSIDERATIONS:
//  1. Until shown it's impossible, if not pointer implemented, basic graph operations will occur around int vector indices
//      rather than named references unless the issue of resizing, reindexing (node_name -> index) is problematic. If possible
//      it will be done in a blackbox fashion to the extent that the user may use either (typing issues if they choose int names...
//      more to think about obviously
//  2. For the time being, this is a course project, or at most, very computation focused, so the implementation will support
//      those core reqs first and foremost. Ad hoc/after-the-fact add and delete are not major concerns. Neither is named
//      node-subgraph analysis. If I ever have use for more db-y type things and for whatever reason don't just use Titan or w/e
//      though the same could be said for boost for the comp stuff.

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
        // create random graph TODO: make sure connected?
        Graph(double target_density = .80, pair<int, int> edge_range = {0, 1}, int size = 10) : target_density(
                target_density), edge_range(edge_range), size(size), graph() {
            for (int i = 0; i < size; ++i) { graph.push_back(Vertex{i}); }
            // Init Mersenne Twister PRNG
            mt19937 gen(rand());
            uniform_int_distribution<> edge_dstr(0, size - 1);
            uniform_real_distribution<> weight_dstr(0.00001, 1.);
            // this may actually produce fewer that required, but at reasonable sizes/densities, should be close enough for a homework
            int max_possible_edges = (size * (size - 1)) / 2;
            int necessary_edges = floor(max_possible_edges * target_density);
            // Generate connections TODO: produces slightly off count due to self connections
            int edge_count{0};
            while (edge_count <= necessary_edges) {
                int vertex_a = edge_dstr(gen);
                int vertex_b = edge_dstr(gen);
                if (vertex_a != vertex_b and not graph[vertex_a].edges.count(vertex_b)) {
                    double weight = weight_dstr(gen);
                    graph[vertex_a].edges[vertex_b] = weight;
                    graph[vertex_b].edges[vertex_a] = weight;
                    edge_count += 1;
                }
            }
        }

        auto neighbors(int vert_id) {
            // lists all nodes y such that there is an edge from x to y. implement overload to accept vertex type?
            return graph[vert_id].edges;
        };

        auto edge_density(int vert_id) {
            // return density of out edges
            // TODO: implement
        }

        auto get_vertices() const {
            return graph;
        };

        inline auto vertex_count() {
            // returns the number of vertices in the graph
            return graph.size();
        };

        inline auto edge_count() {
            // returns the number of edges in the graph
            int count{0};
            for (const auto vertex : graph) {
                count += vertex.edges.size();
            }
            assert(count % 2 == 0); // uDAG should have even edge cardinality
            return count / 2;
        };

    //        auto shortest_path_calc(int source);         friends?

        inline auto are_adjacent(Vertex x, Vertex y) {
            // test whether there is an edge from node x to node y.s
        };
        //add (G, x, y): adds to G the edge from x to y, if it is not there.
        //delete (G, x, y): removes the edge from x to y, if it is there.
        //get_node_value (G, x): returns the value associated with the node x.
        //set_node_value( G, x, a): sets the value associated with the node x to a.
        //get_edge_value( G, x, y): returns the value associated to the edge (x,y).
        //set_edge_value (G, x, y, v): sets the value associated to the edge (x,y) to v.
        // One important consideration for the Graph class is how to represent the graph as a member ADT.
        //  Two basic implementations are generally considered: adjacency list and adjacency matrix depending on
        //  the relative edge density. For sparse graphs, the list approach is typically more efficient, but for
        //  dense graphs, the matrix approach can be more efficient (reference an Algorithm’s source for space
        //  and time analysis). Note in some cases such as add(G, x, y) you may also want to have the edge carry
        //  along its cost. Another approach could be to use (x, y) to index a cost stored in an associated
        //  array or map

    //        auto Graph::shortest_path_calc(int source) {
        auto shortest_path(int source) {
            //    using iset = unordered_set<int>;
            // Takes set of vertices and returns min scoring vertex(edge) using Dijkstra's greedy score
            // THIS COULD BE IMPLEMENTED FROM THE GET GO FOR THE HEAP BY RETURNING THE MIN GREEDY SCORE FOR ALL IT'S EDGES IN X --I THINK, ANYWAY
            // for now, will just use indices... convert for nodes later.
            // assert source and destination are in set
            unordered_set<int> VminX, X, V;
            vector<double> A(size);
            vector<vector<int>> B(size); //TODO: something sorted
            for (int i = 0; i < size; ++i) {
                V.insert(i);
                VminX.insert(i);
            }

            auto update_paths = [](int w_star, int v_star) {
                A[w_star] = 0.;
                X.insert(w_star);
                VminX.erase(VminX.find(w_star));
                B[source] = B[path_to_source].push_back(source);
            };

            auto greedy_criterion_vert = [&](int vert_index) {
                // takes vertices from XminV and returns min dijsktra greedy
                double min{10000000000.}, cost{};
                int v_star{}, w_star{};

                for (auto const &edge : graph[vert_index].edges) {
                    if (not X.count(edge.first)) {
                        cost = A[edge.first] + edge.second;
                        if (cost <= min) {
                            v_star = edge.first;
                            w_star = vert_index;
                        }
                    }
                }

                pair<pair<int, int>, double> optim_edge{{v_star, w_star}, cost};

                return optim_edge;
            };

            A[source] = 0.;
            X.insert(source);
            VminX.erase(VminX.find(source));
            B[source].push_back(source);


            while (X != V) {
                set<int> F{}; // generate frontier set
                for (auto const &vert : X) {
                    for (auto const &edge : graph[source].edges) {
                        if (VminX.count(edge.first) != 0) {
                            F.insert(edge.first);
                        }
                    }
                }
                for (auto const &x : F) {
                    cout << x << endl;
                }
                break;
    //        auto iter_result = dijkstra_greedy_crit(F);
    //            }
            }
        }
};






//        auto shortest_path(int source, int sink) {;}




//        vertices(List): list of vertices in G(V,E).
//        path(u, w): find shortest path between u-w and returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
//        path_size(u, w): return the path cost associated with the shortest path.



ostream &operator<<(ostream &os, const Graph &graph) {
    auto g = graph.get_vertices();
    for (int i{0}; i < g.size(); ++i) {
        os << i << " -> ";
        for (auto &edge : g[i].edges) {
            os << "{" << edge.first << ", " << edge.second << "}, ";
        }
        os << endl;
    }
    return os;
}



//Notes and Reminders:
//
//Write an appropriate set of constructors for each of your classes ensuring proper initialization – especially think about the process for declaring and initializing a graph.
//In this implementation, assume that an edge will have a positive cost function like distance (no negative edge cost).
//Assume the graph edges (E) are undirected.
//Ensure that your ADTs support a graph of at least size 50.
//The random graph procedure should have edge density as a parameter and distance range as a parameter.
//Random graph generator should generate a sufficient set of edges to satisfy the edge density parameter, and each edge should be assigned a randomly generated cost based on the distance range parameter.
//So a graph whose density is 0.1 would have 10% of its edges picked at random and its edge distance would be selected at random from the distance range.
//Compute for a set of randomly generated graphs an average shortest path.



int main() {
    Graph g{};
    cout << setprecision(5);
    cout << g << endl;
//   cout << g.neighbors(93).size() << endl;
    cout << g.vertex_count() << endl;
    cout << g.edge_count() << endl;
    g.shortest_path_calc(0);
    return 0;
}