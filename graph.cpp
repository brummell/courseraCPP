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


class Graph {
    double INF = numeric_limits<double>::infinity();
    int QNAN = numeric_limits<int>::quiet_NaN();
    using min_edge = pair<pair<int, int>, double>;

    struct Vertex {
        int id;
        unordered_map<int, double> edges{{id, 0.0}};
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
    }

    auto find_minimum_spanning_tree() {
    // use Prim's algorithm to find a minimum spanning tree, here using a naive implementation
        // remember, no node gets touched twice
        // TODO: RIGHT NOW ASSUMING CONNECTED COMPONENT
//        using min_edge = pair<pair<int, int>, double>
        auto edge_cmp = [](const min_edge &left, const min_edge &right) { // lift this into higher namespace and reuse accross stl stuff
            if (left.second != right.second) { return left.second < right.second; }
            else if (left.first.first != right.first.first) { return left.first.first < right.first.first; }
            else { return left.first.second < right.first.second; }
        };

        set<int> VminX, X, V;
        vector<double> A(size, INF);
        vector<vector<int>> B(size);
        for (int i = 0; i < size; ++i) {
            V.insert(i); // This can be optimed away probably
            VminX.insert(i);
        }

        VminX.erase(0);
        X.insert(0);
        VminX.insert(0);
        min_edge mini_edge{{QNAN, QNAN}, INF};
        for (auto x : X) {
            for (auto edge : graph[0].edges) {
                if (VminX.count(edge.first and edge.second < mini_edge.second)) {
                    mini_edge.first.first = x; mini_edge.first.second = edge.first; mini_edge.second = edge.second;
                }
            }
        }
        VminX.erase(mini_edge.first.second);
    };

    auto shortest_path_optim() {
        set<int> VminX, X, V;
        vector<double> A(size, INF);
        vector<vector<int>> B(size);
        for (int i = 0; i < size; ++i) {
            V.insert(i); // This can be optimed away probably
            VminX.insert(i);
        }

        auto min_cmp = [](const min_edge &left, const min_edge &right) {
            if (left.second != right.second) { return left.second < right.second; }
            else if (left.first.second != right.first.second) { return left.first.second > right.first.second; }
            else { return left.first.first > right.first.first; }
        };
        set<min_edge, decltype(min_cmp)> vertex_heap(min_cmp);

        auto greedy_criterion = [&](int w) {  // returns min dijsktra greedy score for all edges in for A SINGLE vertex in V-X
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
            return min_edge{{v_star, w_star}, cost};
        };

        auto update_paths = [&](int v_star, int w_star, double cost) {
        //
            A[w_star] = cost;
            X.insert(w_star);
            VminX.erase(VminX.find(w_star));
            B[w_star] = B[v_star];
            B[w_star].push_back(w_star);
        };

        auto update_heap_keys = [&](const int &old_v, const int &old_w, const double &old_cost) {
            for (auto const &w : graph[old_w].edges) {
                if (VminX.count(w.first)) { // is in VminX?, hence edge is on frontier
                    auto found_itr = find_if(begin(vertex_heap), end(vertex_heap),
                                             [&](const min_edge &m) { return m.first.second == w.first;}); //should be an edge
                    double new_cost{w.second + A[old_w]};
                    if (found_itr->second > new_cost) {
                        min_edge new_elem{{old_w, w.first}, new_cost};
                        vertex_heap.erase(found_itr);
                        vertex_heap.insert(new_elem);
                    }
                }
            }
        };

        // Initialize "heap" to hold vertices from V-X and their Djikstra's greedy score
        for (auto const &w : VminX) {
            auto optim_w = greedy_criterion(w); //need to check that vaild result returned
            auto result = vertex_heap.insert(optim_w);
        }

        int w{0};
        auto found_itr = find_if(vertex_heap.begin(), vertex_heap.end(), [&](const min_edge &m) { return m.first.second == w;});
        vertex_heap.insert(min_edge {{found_itr->first.first, found_itr->first.second}, 0});
        vertex_heap.erase(found_itr);
        A[0] = 0;

        while (X != V) {
            auto min_itr = vertex_heap.begin();
            min_edge min{{min_itr->first.first, min_itr->first.second},
                         min_itr->second};
            vertex_heap.erase(min_itr);
            update_paths(min.first.first, min.first.second, min.second);
            update_heap_keys(min.first.first, min.first.second, min.second);
        }

        return pair<decltype(A), decltype(B)> {A,B};
    };

    friend auto run_mc_shortest_path(const int &size, const double &density, const pair<double, double> &range);

    friend ostream &operator<<(ostream &os, const Graph &graph);

};


auto run_mc_shortest_path(const int &size, const double &density, const pair<double, double> &range) {
    cout << "running graph with size: " << size << "   " << "density: " << density << "   "
         << "edge range: " << "(" << range.first << ", "  << range.second << ")" << endl;

    Graph g{density, size, range};
    cout << setprecision(5) << g << endl;

    auto results = g.shortest_path_optim();
    for (int i = 1; i < size; i++) {
        cout << "shortest path to " << i << ": ";
        for (const auto &v : results.second[i]) {
            cout << v << " -> ";
        }
        cout << right << " (" << results.first[i] << ")" << endl;
    }
    cout << "average shortest path length: "
         << accumulate(begin(results.first) + 1, end(results.first), 0) / (size - 1.0) << endl;
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


int main() {
    int size{50};
    pair<double, double> range{0.00001, 10.};

    double density1{0.20};
    run_mc_shortest_path(size, density1, range);

    double density2{0.40};
    run_mc_shortest_path(size, density2, range);

    return 0;
}
