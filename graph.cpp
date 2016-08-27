//
// Created by Brummell, Doug on 8/14/16.
//

//#include "graph.h"
#include <random>
#include <vector>
#include <unordered_map>
//#include <set>
#include <unordered_set>
#include <iomanip>
#include <string>
#include <iostream>
#include <cmath>



using namespace std;

struct edge {
    int vertex;
    double weight;
};

inline bool operator<(const edge &lhs, const edge &rhs) {
    return lhs.vertex < rhs.vertex;
}

//template <typename T>
struct vertex {
    // maybe should be unordered_map
    // possible implement a conversion to int type, possible through name map, to pass vertex more easily to functions
//    T name;
    unordered_set<edge> edges;
};

//    struct path {};
//template <typename T> THIS IS HERE TO ALLOW NODES TO HAVE REFERENCABLE NAMES
class Graph {
    // undirected acyclic graph, strictly positive weights (0==no edge)
    // adjacency list representation for now... binary heap allows faster Dykstra's
    // WILL CREATE DIFFERENT  STRUCTS ETC DEPENDING ON CONSTRUCTION (DENSITY, DIRECTEDNESS, ETC)


    public:
        // constructors

        // create random graph TODO: make sure connected?
        Graph(double target_density = .20, int size = 100) : target_density(target_density), size(size), vertices(size) {
            // Initialize Mersenne Twister pseudo-random number generator
            mt19937 gen(rand());

            // Generate pseudo-random numbers
            // uniformly distributed in range (start, end)
            uniform_int_distribution<> edge_dist(0, size - 1);
            uniform_real_distribution<> weight_dist(0.00001, 1.);

            // this may actually produce fewer that required, but at reasonable sizes/densities, should be close enough for a homework
            int max_possible_edges = (size * (size - 1)) / 2;
            int necessary_edges = floor(max_possible_edges * target_density);

            // Generate connections
            for (int i{0}; i <= necessary_edges; ++i) {
                vertex_map[i] = i;

                int vertex_a = edge_dist(gen);
                int vertex_b = edge_dist(gen);
                double weight = weight_dist(gen);

                edge edge_a_b{vertex_b, weight};
                edge edge_b_a{vertex_a, weight};

                vertices[vertex_a].edges.insert(edge_a_b);
                vertices[vertex_b].edges.insert(edge_b_a);
            }
        }

        // lists all nodes y such that there is an edge from x to y.
        // implement overload to accept vertex type?
        auto neighbors(int vert_id) {
            return vertices[vert_id].edges ;
        }

//        void add_edge(int a, int b, auto weight) {
//            // for now assume the edge will not already exist, later -> if (adjacent(a, b)) {};
//            edge edge_a_b{b, weight};
//            vertices[a].edges.insert(edge_a_b);
//
//            edge edge_b_a{a, weight};
//            vertices[b].edges.insert(edge_b_a);
//        };

//        bool adjacent(int a, int b) {
//            bool are_adjacent{false};
//            int v_a = vertex_map[a], v_b = vertex_map[b];
//            for (const edge &edge : vertices[a].edges) {
//                if (edge.vertex == b) are_adjacent = true;
//            };
//            return are_adjacent;
//        };

//        //Getters and Setters
//        double getx() const { return this->x; } //Add const
//        double gety() const { return this->y; }
//
//        void setx(double v) { x = v; }

        auto get_vertices() const {
            return vertices;
        }

        inline auto vertices_count_temp() {
            return vertices.size();
        }

        auto shortest_path(Graph& g, int source, int dest) {
        // for now, will just use indices... convert for nodes later.
            // assert source and destination are in set
            unordered_set<int> VminX, X, V;
            vector<auto> A(size), B(size);

            dijkstra_greedy_crit = [](edge e) { };
            for (int i = 0; i < size; ++i) {
                V.insert(i);
                VminX.insert(i);
            }
            A[source] = 0.;

            while (VminX != X) {

            }
        }
//        vertices(List): list of vertices in G(V,E).
//        path(u, w): find shortest path between u-w and returns the sequence of vertices representing shorest path u-v1-v2-…-vn-w.
//        path_size(u, w): return the path cost associated with the shortest path.


        private:
            vector<vertex> vertices;
            unordered_map<int, int> vertex_map;
            double target_density;
            int size; // TODO: change name
            // Random seed
            random_device rand;


};

ostream &operator<<(ostream &os, const Graph &graph) {
    auto g = graph.get_vertices();
    for (int i{0}; i < g.size(); ++i) {
        os << i << " -> ";
        for (auto &edge : g[i].edges) {
            os << "{" << edge.vertex << ", " << edge.weight << "}, ";
        }
        os << endl;
    }
    return os;
}



//V (G): returns the number of vertices in the graph
//E (G): returns the number of edges in the graph
//adjacent (G, x, y): tests whether there is an edge from node x to node y.
//neighbors (G, x): lists all nodes y such that there is an edge from x to y.
//add (G, x, y): adds to G the edge from x to y, if it is not there.
//delete (G, x, y): removes the edge from x to y, if it is there.
//get_node_value (G, x): returns the value associated with the node x.
//set_node_value( G, x, a): sets the value associated with the node x to a.
//get_edge_value( G, x, y): returns the value associated to the edge (x,y).
//set_edge_value (G, x, y, v): sets the value associated to the edge (x,y) to v.
//One important consideration for the Graph class is how to represent the graph as a member ADT. Two basic implementations are generally considered: adjacency list and adjacency matrix depending on the relative edge density. For sparse graphs, the list approach is typically more efficient, but for dense graphs, the matrix approach can be more efficient (reference an Algorithm’s source for space and time analysis). Note in some cases such as add(G, x, y) you may also want to have the edge carry along its cost. Another approach could be to use (x, y) to index a cost stored in an associated array or map.

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
//    cout << g << endl;
    cout << g.neighbors(4).size() << endl;
    cout << g.neighbors(93).size() << endl;
    cout << g.neighbors(23).size() << endl;
    cout << g.neighbors(10).size() << endl;
    return 0;
}