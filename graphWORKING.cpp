//
// Created by Brummell, Doug on 8/14/16.
//

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


using namespace std;

// Should encapsulate in namespace
double INF = numeric_limits<double>::infinity();
int QNAN = numeric_limits<int>::quiet_NaN();

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

class Graph {
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

        auto get_vertices() const {
            return graph;
        };


        auto shortest_path_optim() {
            set<int> VminX, X, V;
            vector<double> A(size, INF);
            vector<vector<int>> B(size);
            for (int i = 0; i < size; ++i) {
                V.insert(i);
                VminX.insert(i);
            }

            using min_edge = pair<pair<int, int>, double>;
            auto min_cmp = [](const min_edge &left, const min_edge &right) {
                if (left.second != right.second) { return left.second < right.second; }
                else if (left.first.second != right.first.second) { return left.first.second > right.first.second; }
                else { return left.first.first > right.first.first; }
            };
            set<min_edge, decltype(min_cmp)> vertex_heap(min_cmp); // prog just vert, but then can't cx the path

            auto greedy_criterion = [&](int w) {  // returns min dijsktra greedy score for all edges in the frontier
                double cost{INF};
                int v_star{QNAN}, w_star{w};
                for (auto const &v : graph[w].edges) {
                    if (X.count(v.first) and v.first != w) { // is this in frontier set
                        double tmp_cost = A[v.first] + v.second;
                        if (tmp_cost <= cost) {
                            cost = tmp_cost;
                            v_star = v.first;
                            w_star = w;
                        }
                    }
                }
                cout << "(" << v_star << ", " << w_star << "): " << cost << endl;
                return min_edge{{v_star, w_star}, cost};
            };

            auto update_paths = [&](int v_star, int w_star, double cost) {
                A[w_star] = cost + A[v_star];
                X.insert(w_star);
                VminX.erase(VminX.find(w_star));
                B[w_star] = B[v_star];
                B[w_star].push_back(w_star);
            };

            auto update_heap_keys = [&](const int &old_w, const int &old_v, const double &old_cost) {
                for (auto const &w : graph[old_w].edges) {
                    if (VminX.count(w.first)) { //in VminX, hence edge is on frontier
                        auto found_itr = find_if(begin(vertex_heap), end(vertex_heap),
                                                 [&](const min_edge &m) { return m.first.second == w.first;}); //should be an edge
                        double new_cost{w.second + A[old_w]};
                        if (found_itr->second > new_cost) {
                            cout << "new edge" << old_w << ", " << w.first << ": " << new_cost << endl;
                            min_edge new_elem{{old_w, w.first}, new_cost};
                            vertex_heap.erase(found_itr);
//                            vertex_heap.insert(new_elem);
                        }
                    }
                }
            };

            // build "heap"
            for (auto const &w : VminX) {
                auto optim_w = greedy_criterion(w); //need to check that vaild result returned
                auto result = vertex_heap.insert(optim_w);
            }

            int w{0}; // should be source
            auto found_itr = find_if(vertex_heap.begin(), vertex_heap.end(), [&](const min_edge &m) { return m.first.second == w;});
            vertex_heap.insert(min_edge {{found_itr->first.first, found_itr->first.second}, 0});
            vertex_heap.erase(found_itr);

            update_paths(0, 0, 0); // INF instead of 0?


            for (auto x : vertex_heap) {
                cout << "(" << x.first.first << ", " << x.first.second << "): " << x.second << ", ";
            }
            cout << "first: " << begin(vertex_heap)->first.second << endl << "done" << endl;


            cout << endl;





            for (auto x : X) {
                cout << x << ", ";
                cout << "done" << endl;
            }

//            update_heap_keys(0); // {{0, 0}, 0}

//
            while (X != V) {
                auto min_itr = vertex_heap.begin();
                min_edge min{{min_itr->first.first, min_itr->first.second},
                             min_itr->second}; // {{V1, 0}, 1.3442} <- V1 had lowest score, connected to 0
                cout << "HERE1" << endl;
                vertex_heap.erase(min_itr); // weirdness copy, rework
                cout << "HERE1.5" << endl;
                cout << "edge" << min.first.first << ", " << min.first.second << ": " << min.second << endl;

                update_paths(min.first.first, min.first.second, min.second);
                cout << "HERE2" << endl;
                update_heap_keys(min.first.first, min.first.second, min.second);
                break;
            }

//
            for (auto x : vertex_heap) {
                cout << "(" << x.first.first << ", " << x.first.second << "): " << x.second;
                cout << endl << "done" << endl;

            }
            cout << endl;

            cout << vertex_heap.size();

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
        };
};


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
    Graph g{.80, 10};
    cout << setprecision(5);
    cout << g << endl;
//   cout << g.neighbors(93).size() << endl;
//    cout << g.vertex_count() << endl;
//    cout << g.edge_count() << endl;
    g.shortest_path_optim();
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



// undirected acyclic graph, strictly positive weights (0==no edge)
// adjacency list representation for now... binary heap allows faster Dykstra's
// dumb index version, should use other branch in git with pointer one some day... getter/setters to abstract over both
// WILL CREATE DIFFERENT  STRUCTS ETC DEPENDING ON CONSTRUCTION (DENSITY, DIRECTEDNESS, ETC)
// TODO: Add DAG, UAG, DCG, UCG options, unweighted as well
// TODO: add iterator?

//auto neighbors(int vert_id) {
//    // lists all nodes y such that there is an edge from x to y. implement overload to accept vertex type?
//    return graph[vert_id].edges;
//};
//
//auto edge_density(int vert_id) {
//    // return density of out edges
//    // TODO: implement
//}
//
//inline auto vertex_count() {
//    // returns the number of vertices in the graph
//    return graph.size();
//};
//
//inline auto edge_count() {
//    // returns the number of edges in the graph
//    int count{0};
//    for (const auto vertex : graph) {
//        count += vertex.edges.size();
//    }
//    assert(count % 2 == 0); // uDAG should have even edge cardinality
//    return count / 2;
//};
//
////        auto shortest_path_calc(int source);         friends?
//
//auto minimum_cuts_of_some_sort() {return 0;}
//
//auto connected_component(int source_node) {return 0;} //return new map of new keys pointing to old values? new Graph?
//
//
//inline auto are_adjacent(Vertex x, Vertex y) {
//    // test whether there is an edge from node x to node y.s
//};
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



//        auto shortest_path(int source, int sink) {;}
//        auto generate_all_shortest_paths() {return 0;}
//        vertices(List): list of vertices in G(V,E).
//        path(u, w): find shortest path between u-w and returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
//        path_size(u, w): return the path cost associated with the shortest path.
//
//
//template <typename T>//, decltype(comparator)
//class MinHeap {
//private:
//    vector<T> heap{};
//    auto comp;
//    MinHeap(auto sequence) {
//
//    }
//public:
//    auto min() {
//        return heap.front();
//    };
//
//    void pop_min() {
//        pop_heap(begin(heap), end(heap));
//        heap.pop_back();
//    };
//
//    void push(T elem) {
//        heap.push_back(elem);
//        push_heap(begin(heap), end(heap), comp); //comp?
//    };
//
//    void update_key(T adj_elem) {
//        // vector.erase is linear in adjusted memory, strikes me as inadequate for large graphs
//        // instead find->N(fuck) + swap->1 + pop_back->1 + push_back->1 + push_heap->N)
//        auto old_elem = find(begin(heap), end(heap), adj_elem);
//        auto index = distance(heap.begin(), old_elem);
//        old_elem.second
//        swap(old_elem, end(heap));
//        heap.pop_back();
//        heap.push_back(adj_elem);
//        push_heap(begin(heap), end(heap), comp)
//
//    }
//};


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