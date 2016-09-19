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
#include <queue>
#include <iostream>
#include <algorithm>
#include <cmath>


using namespace std;

struct Vertex {
    // possible implement a conversion to int type, possible through name map, to pass vertex more easily to functions
    int id;
    unordered_map<int, double> edges{{id, 0.0}}; // TODO: should be unordered, once reasonable has defined
    set<int> get_adjacent_nodes() {
        set<int> keys{};
        for (const auto &x : edges) {
            keys.insert(x.first);
        }
        return keys;
    }
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
//  3. Nodes have themselves as a viable edge with 0 weight for now... this may cause a lot of unacceptable overhead when using edges
//       as a standin for viable outnodes, but I like this better at the moment for purity reasons that I may have wrong in my head LULZ

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
        // create random graph TODO: make sure connected?
        Graph(double target_density, int size, pair<int, int> edge_range = {0, 1}) : target_density(
                target_density), edge_range(edge_range), size(size), graph() {
            for (int i = 0; i < size; ++i) { graph.push_back(Vertex{i}); }
            // Init Mersenne Twister PRNG
            mt19937 gen(rand());
            uniform_int_distribution<> edge_dstr(0, size - 1);
            uniform_real_distribution<> weight_dstr(0.00001, 1.);
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

        auto minimum_cuts_of_some_sort() {return 0;}

        auto connected_component(int source_node) {return 0;} //return new map of new keys pointing to old values? new Graph?


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


        auto recompute_heap_key (const Vertex& vertex) {
            auto new_edges = vertex.edges;
            return new_edges;
        };

        auto shortest_path_optim() {
            set<int> VminX, X, V;
            vector<double> A(size);
            vector<vector<int>> B(size); //TODO: something sorted
            for (int i = 0; i < size; ++i) {
                V.insert(i);
                VminX.insert(i);
            }

            using min_edge = pair<pair<int, int>, double>;
            auto min_cmp = [](const min_edge& left, const min_edge& right) { return left.second > right.second; };
            priority_queue<int, std::vector<int>, decltype(min_cmp)> vertex_heap(min_cmp); // prog just vert, but then can't cx the path

            auto greedy_criterion = [&](int w) {  // returns min dijsktra greedy score for all edges in the frontier
                double min{10000000000.}, cost{};
                int v_star{}, w_star{};
                for (auto const &v : graph[w].edges) {
                    if (X.count(v.first) and v.first != w) { // is this in frontier set
                        cost = A[v.first] + v.second;
                        if (cost <= min) { min = cost; v_star = v.first; w_star = w; }
                    }
                }
                return min_edge{{v_star, w_star}, cost};
            };

            auto update_paths = [&](int v_star, int w_star) {
                A[w_star] = graph[w_star].edges[v_star] + A[v_star];
                X.insert(w_star);
                VminX.erase(VminX.find(w_star));
                B[w_star] = B[v_star];
                B[w_star].push_back(w_star);

                update_heap();
            };

            update_paths(0, 0);

            for (auto const &w : VminX) {
                min_edge =
                vertex_heap.push()
            }

        };

        auto shortest_path(int source) {
            using min_edge = pair<pair<int, int>, double>;

            set<int> VminX, X, V;
            vector<double> A(size);
            vector<vector<int>> B(size); //TODO: something sorted
            for (int i = 0; i < size; ++i) {
                V.insert(i);
                VminX.insert(i);
            }

            auto update_paths = [&](int v_star, int w_star) {
                A[w_star] = graph[w_star].edges[v_star] + A[v_star];
                X.insert(w_star);
                VminX.erase(VminX.find(w_star));
                B[w_star] = B[v_star];
                B[w_star].push_back(w_star);
            };

            auto greedy_criterion_vert = [&](int w) {
                // takes vertices from XminV and returns min dijsktra greedy score
                double min{10000000000.}, cost{};
                int v_star{}, w_star{};

                for (auto const &v : graph[w].edges) {
                    if (X.count(v.first) and
                        v.first != w) { // is this v or a node in V-X in X yet, equivalent of generating frontier set
                        cost = A[v.first] + v.second;
                        if (cost <= min) {
                            min = cost;
                            v_star = v.first;
                            w_star = w;
                        }
                    }
                }
                return min_edge{{v_star, w_star}, cost};
            };


            // initialize path records and sets for source; seems you can call it with v* and w* both being the source
            //   but the current implementation requires the graph implement to know dist(a, a) = 0... which it currently
            //   does not... also, can empty vector manipulations are not well-understood by a doug at this point... HASKELL!!!!!
            update_paths(0, 0);

            while (X != V) {
                // generate frontier set explicitly?
                vector<min_edge> candidate_edges{};
                for (auto const &vert : VminX) {
                    set<int> intersection{};
                    auto outnodes = graph[vert].get_adjacent_nodes();
                    set_intersection(begin(outnodes), end(outnodes), begin(X), end(X),
                                     inserter(intersection, begin(intersection)));
                    if (intersection.size() != 0) {
                        candidate_edges.push_back(greedy_criterion_vert(vert));
                        for (auto v : candidate_edges) { cout << v.first.first << " " << v.first.second << ", "; }
                    }

                }
                auto optim_edge = min_element(begin(candidate_edges), end(candidate_edges),
                                              [](const min_edge &e1, const min_edge &e2) {
                                                  return e1.second < e2.second;
                                              });
                update_paths(optim_edge->first.first, optim_edge->first.second);

            }

            cout << "A" << endl;
            for (auto x : A) { cout << x << ", "; }
            cout << endl;
            cout << endl;

            cout << "B" << endl;
            int i = 0;
            for (auto b : B) {
                cout << i << ": ";
                for (auto x : b) { cout << x << ", "; }
                i += 1;
            }
        }
};


//        auto shortest_path(int source, int sink) {;}
//        auto generate_all_shortest_paths() {return 0;}


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

int main() {
    Graph g{.10, 250};
    cout << setprecision(5);
//    cout << g << endl;
//   cout << g.neighbors(93).size() << endl;
    cout << g.vertex_count() << endl;
    cout << g.edge_count() << endl;
    g.shortest_path(0);
    return 0;
}



// for debugging only
//cout << "V" << endl;
//for (auto v : V) { cout << v << ", "; }
//cout << endl;
//cout << endl;
//cout << "VminX" << endl;
//for (auto x : VminX) { cout << x << ", "; }
//cout << endl;
//cout << endl;
//cout << "X" << endl;
//for (auto x : X) { cout << x << ", "; }
//cout << endl;
//cout << endl;
//
//cout << "A" << endl;
//for (auto x : A) { cout << x << ", "; }
//cout << endl;
//cout << endl;
//
//cout << "B" << endl;
//int i = 0;
//for (auto b : B) {
//cout << i << ": ";
//for (auto x : b) { cout << x << ", "; }
//i += 1;