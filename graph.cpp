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

using namespace std;

struct Vertex {
    int id;
    unordered_map<int, double> edges{{id, 0.0}};
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
    unordered_map<int, int> index_map; // template for when node names aren't just the integers of where they were placed
    vector<Vertex> graph;
    unordered_map<int, int> vertex_map;
    pair<int, int> edge_range;
    double target_density;
    int size;
    // Random seed
    random_device rand;

public:
    // constructor to create random graph
    Graph(double target_density, int size, pair<double, double> edge_range) : target_density(target_density),
                                                                              size(size),
                                                                              edge_range(edge_range),
                                                                              graph() {
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

    auto get_vertices() const {
        return graph;
    };

    auto shortest_path(int source) {
        using min_edge = pair<pair<int, int>, double>;
        set<int> VminX, X, V;
        vector<double> A(size);
        vector<vector<int>> B(size);
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

        update_paths(0, 0);

        while (X != V) {
            vector<min_edge> candidate_edges{};
            for (auto const &vert : VminX) {
                set<int> intersection{};
                auto outnodes = graph[vert].get_adjacent_nodes();
                set_intersection(begin(outnodes), end(outnodes), begin(X), end(X),
                                 inserter(intersection, begin(intersection)));
                if (intersection.size() != 0) {
                    candidate_edges.push_back(greedy_criterion_vert(vert));
                }
            }
            auto optim_edge = min_element(begin(candidate_edges), end(candidate_edges),
                                          [](const min_edge &e1, const min_edge &e2) {
                                              return e1.second < e2.second;
                                          });
            update_paths(optim_edge->first.first, optim_edge->first.second);
        }
        return A;
    }
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
    Graph g{.50, 500, pair<double, double>{1.0,10.0}};
    auto paths = g.shortest_path(0);

//    cout << g << endl;
//    Graph g{.20, 50};

    return 0;
}