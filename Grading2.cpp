#include <iostream>
#include <iomanip>
#include <ctime>
#include <cstdlib>

using namespace std;

const int maxnum = 100;
const int maxint = 9999;


bool is_connected(bool *graph[],int size){
    int old_size=0,c_size=0;

    bool* close = new bool[size];
    bool* open = new bool[size];

    for(int i=1;i<=size;++i)
        open[i]=close[i]=false;
    open[1] = true;

    while(c_size < size){
        for(int i=1;i<=size;++i){
            old_size=c_size;
            if(open[i]&&(close[i]==false)){
                close[i]=true;
                c_size++;

                for(int j=1;j<=size;++j)
                    open[j]=open[j]||graph[i][j];
            }
        }
        if(c_size==size) {
            //cout<<"The graph is connected."<<endl;
            return true;
        }
        if(old_size==c_size) {
            //cout<<"The graph is not connected"<<endl;
            return false;
        }
    }
}

//function prob: simple function that can produce random number from a certain range. 
double prob(double start, double end)
{
    return start+(end-start)*rand()/(RAND_MAX + 1.0);
}

//Dijkstra: main function to compute the shortest path in the graph. 
void Dijkstra(int n, int v, int *dist, int *prev, int c[maxnum][maxnum])
{
    bool s[maxnum];    // if s is already in the set
    for(int i=1; i<=n; ++i)
    {
        dist[i] = c[v][i];
        s[i] = 0;     // initialization
        if(dist[i] == maxint)
            prev[i] = 0;
        else
            prev[i] = v;
    }
    dist[v] = 0;
    s[v] = 1;

    // put the shortest path in dist[] and put the point in S. 
    //When all points are in s, dist[] have the shortest paths from 1 to the other points.
    for(int i=2; i<=n; ++i)
    {
        int tmp = maxint;
        int u = v;
        // get the shortest path
        for(int j=1; j<=n; ++j)
            if((!s[j]) && dist[j]<tmp)
            {
                u = j;              // u£ºpoint with the shortest path 
                tmp = dist[j];
            }
        s[u] = 1;    // put u in s

        // refresh dist
        for(int j=1; j<=n; ++j)
            if((!s[j]) && c[u][j]<maxint)
            {
                int newdist = dist[u] + c[u][j];
                if(newdist < dist[j])
                {
                    dist[j] = newdist;
                    prev[j] = u;
                }
            }
    }
}

void searchPath(int *prev,int v, int u)
{
    int que[maxnum];
    int tot = 1;
    que[tot] = u;
    tot++;
    int tmp = prev[u];
    while(tmp != v)
    {
        que[tot] = tmp;
        tot++;
        tmp = prev[tmp];
    }
    que[tot] = v;
    for(int i=tot; i>0; --i)
        if(i != 1)
            cout << que[i] << " -> ";
        else
            cout << que[i] << endl;
}

int main()
{
    int size;                // size of the graph
    int dist[maxnum];        // the shortest path to a certain point
    int prev[maxnum];        // record the last point 
    int c[maxnum][maxnum];   // graph with length 
    double possible;         // the density of the graph 
    double p,q;				 // range of the distance
    cout<<"¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤"<<endl;
    cout<<"The program is to show how Dijkstra works. By input the density and the size of the graph,"<<endl;
    cout<<"the program will print out the whole graph for you to make the result more intuitive."<<endl;
    cout<<"Meanwhile, it will give you the average paht length and the shortest path from 1 to the size you set."<<endl;
    cout<<"By coding this program, I have a better understanding of how Dijkstra works and know more about C++ functions"<<endl;
    cout<<"The outcome may be not too outstanding, it do take my a long time to finish the task."<<endl;
    cout<<""<<endl;
    cout<<"Hope you enjoy it!"<<endl;
    cout<<""<<endl;
    cout<<"Copyright: Andrea Chen" <<endl;
    cout<<"¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤¡¤"<<endl;

    cout<<""<<endl;
    // guideline
    cout<<"Please input the density:";
    cin>>possible;

    cout<<"Please input the size of the graph:";
    cin>>size;

    cout<<"Please input the range of the distance:";
    cin>> p >> q;
    bool** graph;
    srand(time(0));
    graph = new bool*[size];
    for(int i=1;i<=size;++i)
        graph[i] = new bool[size];

    //density
    for(int i=1;i<=size;++i)
        for(int j=i;j<=size;++j)
            if(i==j) graph[i][j]=false;
            else {
                graph[i][j]= graph[j][i] = (prob(0.0,1.0)<possible);
            }

    //determine whether the graph is connected
    is_connected(graph,size);

    if(is_connected(graph,size)==true){
        cout<<" "<<endl;
        cout<<"The graph is connected."<<endl;
        cout<<" "<<endl;


        // Initialization
        for(int i=1; i<=size; ++i)
            for(int j=1; j<=size; ++j)
                c[i][j] = maxint;

        //set the graph
        for(int i=1; i<=size; ++i)
            for(int j=i; j<=size;++j)
            {
                if(graph[i][j]==true)
                {
                    c[i][j] = c[j][i] = prob(p,q);     // undirected graph and give the edge a positive length range from 1-10
                }
            }

        for(int i=1; i<=size; ++i)
            dist[i] = maxint;
        for(int i=1; i<=size; ++i)
        {
            for(int j=1; j<=size; ++j)
                cout<<std::left<<setw(8)<<c[i][j];
            cout<<endl;
        }

        //calculate the average path length
        int temp = 0;
        for(int i=2;i<=size;++i)
        {
            Dijkstra(i, 1, dist, prev, c);
            if(dist[i]==maxint)  dist[i]=0;
            cout<<""<<endl;
            cout<<"dist= "<<dist[i]<<endl;
            temp=temp + dist[i];
        }
        cout.precision(3);
        cout<<" "<<endl;
        cout<<"The average path length ="<<temp/(size-1)<<endl;

        Dijkstra(size, 1, dist, prev, c);


        // This is an alternative way to calculate the average path length which is simpler.
        /* for(int i=2;i<=size;++i)
        {
               if(dist[i]==maxint)  dist[i]=0;
               cout<<"dist="<<dist[i]<<endl;
               temp=temp + dist[i];
       }
        cout.precision(3);
        cout<<"The average path length ="<<temp/(size-1)<<endl;*/


        //Printout the shortest length from 1 to size
        cout<<" "<<endl;
        cout << "The shortest length for the whole graph is : " << dist[size] << endl;

        // Show the shortest path
        cout<<" "<<endl;
        cout << "The shortest path is: ";
        searchPath(prev, 1, size);
    }
    else
        cout<<"The graph is not connected"<<endl;

}