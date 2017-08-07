#include <stdlib.h>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <stack>
#include <tuple>
#include <vector>

/**
 *  @brief A* Search Algorithm
 *
 *  @param nodes        Number of nodes
 *  @param graph        2-dimension ajacent matrix
 *  @param initial      Starting node
 *  @param goal         Target node
 *  @param estimated    Heuristic function
 *
 *  @note  If estimated function is not provided, A* search is equivalent to uniformed-cost search
 */
template <typename Weight>
std::vector<size_t> Astar(size_t nodes, const Weight ** graph, const size_t initial, const size_t goal, const std::function<Weight(const size_t current)>& estimated = [](const size_t current) -> Weight { return 0; }) {
    // return an empty std::vector on invaild input
    if (nodes == 0 || graph == nullptr) return std::vector<size_t>();

    // return <initial, goal> if initial equals to goal
    if (initial == goal) return std::vector<size_t>(initial, goal);

    // state: current node, via, g(n), h(n)
    // g(n):  cost so far to reach n
    // h(n):  estimated cost to goal from n
    using state = std::tuple<size_t, size_t, Weight, Weight>;

    // compare function for std::priority_queue
    auto cmp = [](const state& a, const state& b) {
        // (g(a) + h(a)) > (g(b) + h(b))
        // a.k.a sort by estimated total cost from initial node to goal through n
        return ((std::get<2>(a) + std::get<3>(a)) > (std::get<2>(b) + std::get<3>(b)));
    };

    // frontier, increasing order
    std::priority_queue<state, std::vector<state>, decltype(cmp)> frontier(cmp);

    // visited node
    std::set<size_t> visited;

    // froniter set
    std::set<size_t> froniter_set;

    // record trace
    size_t * visit = (size_t *)malloc(sizeof(size_t) * nodes);

    // start from initial node
    frontier.push({initial, initial, 0, estimated(initial)});
    froniter_set.emplace(initial);

    // a flag variable
    bool found = false;

    // frontier will be empty if goal is not in our search space
    while (!frontier.empty()) {
        // this is least weighted node
        auto current = frontier.top();

        // node number
        size_t current_node = std::get<0>(current);

        frontier.pop();
        froniter_set.erase(current_node);

        // record that this node has been visited
        visited.emplace(current_node);

        // we go to this node via last
        size_t from = std::get<1>(current);
        visit[current_node] = from;

        // goal test
        if ((found = (current_node == goal))) {
            break;
        } else {
            // iterate possible node
            for (size_t to = 0; to < nodes; to++) {
                // don't stay here
                // and if node to is reachable from current node
                if (to != current_node && graph[current_node][to] != std::numeric_limits<Weight>::max() && (visited.find(to) == visited.end())) {
                    // don't go back
                    if (froniter_set.find(to) == froniter_set.end()) {
                        // push this state to frontier
                        frontier.push({to, current_node, std::get<2>(current) + graph[current_node][to], estimated(to)});
                        froniter_set.emplace(to);
                    }
                }
            }
        }
    }

    std::vector<size_t> trace;
    if (found) {
        // build trace
        std::stack<size_t> trace_stack;
        size_t from = goal;
        trace_stack.push(from);
        while (from != initial) {
            from = visit[from];
            trace_stack.push(from);
        }

        // move trace into std::vector
        while (!trace_stack.empty()) {
            trace.emplace_back(trace_stack.top());
            trace_stack.pop();
        }
    }
    free((void *)visit);

    return trace;
}

#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

int main(int argc, const char * argv[]) {
    freopen("demo.txt", "r", stdin);

    int N;
    std::cin >> N;

    std::vector<std::string> nodes;
    std::string node;
    for (int n = 0; n < N; n++) {
        std::cin >> node;
        nodes.emplace_back(node);
    }

    std::map<std::string, int> nodes_label;
    int label = 0;
    for (auto &node : nodes) {
        nodes_label[node] = label++;
    }

    int ** graph = new int*[N];
    for (int src = 0; src < N; src++) {
        graph[src] = new int[N];
        for (int dest = 0; dest < N; dest++) {
            if (src == dest) {
                graph[src][dest] = 0;
            } else {
                graph[src][dest] = std::numeric_limits<int>::max();
            }
        }
    }

    int connections;
    std::cin >> connections;
    std::string src, dest;
    int weight;
    for (int i = 0; i < connections; i++) {
        std::cin >> src >> dest >> weight;
        graph[nodes_label[src]][nodes_label[dest]] = weight;
        graph[nodes_label[dest]][nodes_label[src]] = weight;
    }

    std::string initial, goal;
    std::cin >> initial >> goal;

    std::map<int, int> estimated;
    std::string from;
    int h, value;
    std::cin >> h;
    for (int e = 0; e < h; e++) {
        std::cin >> from >> value;
        estimated[nodes_label[from]] = value;
    }

    std::vector<size_t> trace = Astar<int>(N, (const int **)graph, nodes_label[initial], nodes_label[goal]);

    std::cout << initial << " -> " << goal << ":\n";
    std::for_each(trace.begin(), trace.end(), [&nodes](const size_t node) {
        std::cout << nodes.at(node) << ' ';
    });
    std::cout << '\n';

    for (int i = 0; i < N; i++) {
        delete [] graph[i];
    }
    delete [] graph;
    return 0;
}
