#include <iostream>
#include <ctime>
#include <cstdlib>
#include <vector>
#include <queue>    //priority_queue

const int NODES = 50;

using namespace std;

class Graph
{
private:

    //double is for the weight, int for the name of the vertex
    typedef pair<double,int> ii;

    //we store the graph as an adjacency list
    //each vertex points to a list of vertex adjacent to it
    //we also store the corresponding weight value
    vector< vector<ii> > edge;

    //Q can store the vertices and their distance from the source vertex
    priority_queue<ii, vector<ii>, greater<ii> > Q;

    //we store the shortest paths in an array
    vector<double> distance;

    //V shall be initialized to store the number of vertices in the graph
    //adjacent_vertices shall be used to count the number of vertices that
    //can be connected to the source vertex
    int V, adjacent_vertices;

    //MAX_DISTANCE shall be initialized to store the maximum possible distance
    //sum shall store the sum of the weights of all shortest paths from source
    double MAX_DISTANCE, sum;

public:
    Graph(int vertices=-1)
    {
        V = vertices;

        //we obtain the value of MAX_DISTANCE from the value of INT_MAX
        //which is found in the limits.h header file

        MAX_DISTANCE=static_cast<double>(INT_MAX);

        //an array of size V+1 is created with each index initialized to MAX_DISTANCE
        //we use V+1 so that the vertices can be numbered from 1 to V

        distance=vector<double> (V+1,MAX_DISTANCE);

        //the adjacency list is resized to fit all vertices
        edge.resize(V+1);

    }


    void addEdge(int u, int v, double w)
    {
        //since its an undirected graph, we add the corresponding adjacent vertices
        //edges to the list of each vertex

        edge[u].push_back(ii(w,v));
        edge[v].push_back(ii(w,u));
    }

    void monteCarlo(double density, double weightLO, double weightHI){
        //probability stores the value of probability of an edge being present
        //weight stores the random value of weight generated

        double probability,weight;

        //i runs from 1 to 50
        //j=i+1 to 50 ensures that no edge is repeated

        for(int i = 1;i <= NODES; ++i){
            for(int j= i+1; j <= NODES; ++j){

                //value of rand() lies between 0.0 to RAND_MAX
                //hence probability always lies between 0.0 to 1.0

                probability = static_cast<double>(rand())/RAND_MAX;

                //if probability value satisfies the density condition
                //then we add a randomly chosen weight to the graph
                //the weight shall vary from weightLO to WeightHI

                if(probability <= density){
                    weight=weightLO+(weightHI-weightLO)*(static_cast<double>(rand())/RAND_MAX);
                    addEdge(i,j,weight);
                }
            }
        }
    }

    //computes the average shortest path

    double averageDistance()
    {
        //initially the sum of all edge weights of the shortest paths from the source is zero
        sum=0;

        adjacent_vertices=V;

        //the number of adjacent vertices to the source vertex is V-1
        for(int i=2;i<=V;++i)
        {
            //if a vertex is not connected to the source vertex
            //then the count is decremented
            //otherwise the distance of the vertex from the source is updated

            if(distance[i]==MAX_DISTANCE) adjacent_vertices--;
            else sum+=distance[i];
        }

        //we return the average shortest path
        return sum/adjacent_vertices;
    }

    //returns the average shortest path
    double dijkstra(int source)
    {
        //the source vertex will have a distance 0.0
        distance[source]=0.0;

        //source vertex pushed to stack
        Q.push(ii(0.0,source));


        //iterate until the queue is empty
        while(!Q.empty())
        {
            //the vertex with the minimum distance from the source is popped
            ii top=Q.top();
            cout << "(" << top.first << ", " << top.second << ")" << endl;
            Q.pop();

            double d=top.first;
            int v=top.second;

            //we analyze each vertex only once.
            //The occurences of the vertices with greater distance
            //should not be processed again

            if(d<=distance[v])
            {
                //all neighbouring vertices of 'v' are checked
                for(unsigned int i=0;i<edge[v].size();++i)
                {
                    int v2=edge[v][i].second;
                    double d2=edge[v][i].first;

                    //if the earlier distance was more, then we update the distance array
                    //and push the vertex onto the queue
                    //if the vertex was already present on the queue, then
                    //the if(d<=distance[v]) prevents reprocessing the vertex with larger distance

                    if(distance[v2]>distance[v]+d2)
                    {
                        distance[v2]=distance[v]+d2;
                        Q.push(ii(distance[v2],v2));
                    }
                }
            }
        }

        //returns the average distance of all vertices from the source vertex
        return averageDistance();
    }

    void reset()
    {
        //we reset all the data structures so that the same object can be reused

        priority_queue<ii, vector<ii>, greater<ii> > empty;
        swap(Q,empty);
        distance.clear();
        edge.clear();
    }

};

int main()
{
    cout << "Enter number of trials :";
    int trials;
    cin >> trials;

    //create a random number
    srand(time(0));

    double sum20 = 0,sum40 = 0;

    for(int i = 0; i < trials; i++){
        Graph G20 = Graph(50);
        Graph G40 = Graph(50);
        G20.monteCarlo(0.20,1.0,10.0);
        G40.monteCarlo(0.40,1.0,10.0);
        sum20+=G20.dijkstra(1);
        sum40+=G40.dijkstra(1);
        G20.reset();
        G40.reset();
    }

    cout<<"Average Shortest Path for a Graph with 20% Density and edge weights between 1.0 to 10.0 is: "<<sum20/trials<<endl;
    cout<<"Average Shortest Path for a Graph with 40% Density and edge weights between 1.0 to 10.0 is: "<<sum40/trials<<endl;

    return 0;
}