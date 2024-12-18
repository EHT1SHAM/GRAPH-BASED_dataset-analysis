#include <iostream>
#include <fstream>
#include <unordered_map>
#include <queue>
#include <climits>
#include <algorithm>

using namespace std;

class Graph
{
public:
    unordered_map<int, vector<pair<int, int>>> adjList;

    void addEdge(int u, int v, int weight)
    {
        adjList[u].push_back({v, weight});
        adjList[v].push_back({u, weight}); // Undirected graph
    }

    vector<pair<int, int>> getNeighbors(int node)
    {
        return adjList[node];
    }

    int getHeuristic(int node)
    {
        return adjList[node].size(); // Number of direct neighbors
    }

    void printGraph()
    {
        for (const auto &pair : adjList)
        {
            cout << pair.first << " -> ";
            for (const auto &neighbor : pair.second)
            {
                cout << "(" << neighbor.first << ", " << neighbor.second << ") ";
            }
            cout << endl;
        }
    }
};

// Load the graph from a file
Graph loadGraphFromFile(const string &filename)
{
    Graph g;
    ifstream file(filename);

    if (!file)
    {
        cerr << "Error: Could not open the file " << filename << endl;
        exit(EXIT_FAILURE);
    }

    int node1, node2, weight;
    while (file >> node1 >> node2 >> weight)
    {
        g.addEdge(node1, node2, weight);
    }
    file.close();
    return g;
}

// Dijkstra's Algorithm
pair<vector<int>, int> dijkstra(Graph &g, int start, int goal)
{
    unordered_map<int, int> dist; //used for referance to reconstruct
    unordered_map<int, int> parent;
    for (auto &node : g.adjList)
    {
        dist[node.first] = INT_MAX;
    }
    dist[start] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    pq.push({0, start});

    while (!pq.empty())
    {
        int currDist = pq.top().first;
        int currNode = pq.top().second;
        pq.pop();

        if (currNode == goal)
            break;

        for (auto &neighbor : g.getNeighbors(currNode))
        {
            int nextNode = neighbor.first;
            int weight = neighbor.second;

            if (currDist + weight < dist[nextNode])
            {
                dist[nextNode] = currDist + weight;
                parent[nextNode] = currNode;
                pq.push({dist[nextNode], nextNode});
            }
        }
    }

    vector<int> path;
    int node = goal;
    while (node != start)
    {
        path.push_back(node);
        node = parent[node];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return {path, dist[goal]};
}

// A* Algorithm
pair<vector<int>, int> aStar(Graph &g, int start, int goal)
{
    unordered_map<int, int> gScore;
    unordered_map<int, int> fScore;
    unordered_map<int, int> parent;
    for (auto &node : g.adjList)
    {
        gScore[node.first] = INT_MAX;
        fScore[node.first] = INT_MAX;
    }
    gScore[start] = 0;
    fScore[start] = g.getHeuristic(start);

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<>> pq;
    pq.push({fScore[start], start});

    while (!pq.empty())
    {
        int currNode = pq.top().second;
        pq.pop();

        if (currNode == goal)
            break;

        for (auto &neighbor : g.getNeighbors(currNode))
        {
            int nextNode = neighbor.first;
            int weight = neighbor.second;

            int tentativeGScore = gScore[currNode] + weight;
            if (tentativeGScore < gScore[nextNode])
            {
                gScore[nextNode] = tentativeGScore;
                fScore[nextNode] = gScore[nextNode] + g.getHeuristic(nextNode);
                parent[nextNode] = currNode;
                pq.push({fScore[nextNode], nextNode});
            }
        }
    }

    vector<int> path;
    int node = goal;
    while (node != start)
    {
        path.push_back(node);
        node = parent[node];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return {path, gScore[goal]};
}

// Main function with menu
int main()
{
    string filename = "social-network-proj-graph.txt"; // Update with your file name
    Graph graph = loadGraphFromFile(filename);

    while (true)
    {
        cout << "\n--- Shortest Path Menu ---\n";
        cout << "1. Find shortest path using Dijkstra's Algorithm\n";
        cout << "2. Find shortest path using A* Algorithm\n";
        cout << "3. print graph\n";
        cout << "4. Exit\n";
        cout << "Enter your choice: ";
        int choice;
        cin >> choice;

        if (choice == 1)
        {
            int start, goal;
            cout << "Enter the starting node: ";
            cin >> start;
            cout << "Enter the goal node: ";
            cin >> goal;

            auto result = dijkstra(graph, start, goal);
            cout << "Shortest path using Dijkstra: ";
            for (int node : result.first)
                cout << node << " ";
            cout << "\nCost: " << result.second << endl;
        }
        else if (choice == 2)
        {
            int start, goal;
            cout << "Enter the starting node: ";
            cin >> start;
            cout << "Enter the goal node: ";
            cin >> goal;

            auto result = aStar(graph, start, goal);
            cout << "Shortest path using A*: ";
            for (int node : result.first)
                cout << node << " ";
            cout << "\nCost: " << result.second << endl;
        }
        else if (choice == 3)
        {
            graph.printGraph();
        }
        else if (choice == 4)
        {
            cout << "Exiting the program...\n";
            break;
        }
        
        else
        {
            cout << "Invalid choice. Please try again.\n";
        }
    }
    return 0;
}