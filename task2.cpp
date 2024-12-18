#include <iostream>
#include <fstream>
#include <list>
#include <unordered_map>
#include <algorithm>

using namespace std;

// Graph representation using adjacency list
class Graph {
public:
    unordered_map<int, list<pair<int, int>>> adjList;

    void addEdge(int u, int v, int weight) {
        adjList[u].push_back({v, weight});
        adjList[v].push_back({u, weight}); // Undirected graph
    }

    list<pair<int, int>> getNeighbors(int node) {
        return adjList[node];
    }
};

// Load the graph from a file
Graph loadGraphFromFile(const string& filename) {
    Graph g;
    ifstream file(filename);

    if (!file) {
        cerr << "Error: Could not open the file " << filename << endl;
        exit(EXIT_FAILURE);
    }

    int node1, node2, weight;
    while (file >> node1 >> node2 >> weight) {
        g.addEdge(node1, node2, weight);
    }
    file.close();
    return g;
}

// Load the influence scores from a file
unordered_map<int, int> loadInfluenceScores(const string& filename) {
    unordered_map<int, int> influenceScores;
    ifstream file(filename);

    if (!file) {
        cerr << "Error: Could not open the file " << filename << endl;
        exit(EXIT_FAILURE);
    }

    int user, score;
    while (file >> user >> score) {
        influenceScores[user] = score;
    }
    file.close();
    return influenceScores;
}

// Find the longest increasing path in terms of influence scores
pair<int, list<int>> findLongestIncreasingPath(Graph& g, unordered_map<int, int>& influenceScores) {
    unordered_map<int, int> dp;
    unordered_map<int, int> parent;
    list<int> nodes;

    for (const auto& pair : influenceScores) {
        nodes.push_back(pair.first);
    }

    nodes.sort([&](int a, int b) {
        return influenceScores[a] < influenceScores[b];
    });

    int maxLength = 0;
    int endNode = -1;

    for (int node : nodes) {
        dp[node] = 1;
        parent[node] = -1;

        for (const auto& neighborPair : g.getNeighbors(node)) {
            int neighbor = neighborPair.first;
            if (influenceScores[neighbor] < influenceScores[node] && dp[neighbor] + 1 > dp[node]) {
                dp[node] = dp[neighbor] + 1;
                parent[node] = neighbor;
            }
        }

        if (dp[node] > maxLength) {
            maxLength = dp[node];
            endNode = node;
        }
    }

    list<int> path;
    while (endNode != -1) {
        path.push_front(endNode);
        endNode = parent[endNode];
    }

    return {maxLength, path};
}

// Main function
int main() {
    string graphFilename = "social-network-proj-graph.txt";
    string influenceFilename = "social-network-proj-influences.txt";

    Graph graph = loadGraphFromFile(graphFilename);
    unordered_map<int, int> influenceScores = loadInfluenceScores(influenceFilename);

    auto result = findLongestIncreasingPath(graph, influenceScores);
    int maxLength = result.first;
    list<int> path = result.second;

    cout << "Maximum length of the chain: " << maxLength << endl;
    cout << "Sequence of users in the chain: ";
    for (int user : path) {
        cout << user << " ";
    }
    cout << endl;

    return 0;
}