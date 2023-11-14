#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <set>
#include <cmath>

using namespace std;

namespace a_star
{
  struct Point {
    int x, y;

    Point() : x(0), y(0) {}
    Point(int px, int py) : x(px), y(py) {}
  };

  struct Node {
    Point point;
    int cost;
    int heuristic;

    Node(const Point& p, int c, int h) : point(p), cost(c), heuristic(h) {}
  };

  bool operator<(const Node& lhs, const Node& rhs) {
    return lhs.cost + lhs.heuristic < rhs.cost + rhs.heuristic;
  }

  struct Compare {
    using is_transparent = void;  // important

    bool operator() (const Node& lhs, const Node& rhs) const {
      return lhs.cost + lhs.heuristic < rhs.cost + rhs.heuristic;
    }
  };

  int calculateHeuristic(const Point& a, const Point& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
  }

bool astarSearch(const vector<vector<int>>& grid, const Point& start, const Point& goal, vector<Point>& path) {
  int rows = grid.size();
  int cols = grid[0].size();

  // Define possible movement directions
  int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
  int dy[] = {0, 0, -1, 1, 1, -1, 1, -1};


  // Create a priority queue for the open list
  multiset<Node, Compare> openSet;

  // Create a 2D vector to track visited nodes
  vector<vector<bool>> visited(rows, vector<bool>(cols, false));

  // Create a 2D vector to store parent information for tracing the path
  vector<vector<Point>> parent(rows, vector<Point>(cols, { -1, -1 }));

  // Initialize the start node
  Node startNode(start, 0, calculateHeuristic(start, goal));
  openSet.insert(startNode);

  int cnt = 0;
  while (!openSet.empty()) {

    // std::cout << "\n cnt: " << cnt << std::endl;
    // for (auto it = openSet.begin(); it != openSet.end(); it++) {
    //   std::cout << "node (" << it->point.x << ", " << it->point.y << "), cost " << it->cost + it->heuristic << std::endl;
    // }


    // Get the node with the lowest total cost from the set
    Node current = *openSet.begin();

    // std::cout << "current node (" << current.point.x << ", " << current.point.y << ")" << std::endl;

    openSet.erase(openSet.begin());

    // Check if the current node is the goal
    if (current.point.x == goal.x && current.point.y == goal.y) {
      // cout << "Goal reached! Total cost: " << current.cost << endl;

      // std::cout << "cnt: " << cnt << std::endl;

      path.resize(0);
      Point p = goal;
      path.push_back(p);

      while (!(p.x == start.x && p.y == start.y)) {
        p = parent[p.x][p.y];
        path.push_back(p);
      }

      std::reverse(path.begin(), path.end());

      return true;
    }

    // Mark the current node as visited
    visited[current.point.x][current.point.y] = true;

    // Explore neighbors
    for (int i = 0; i < 8; ++i) {
      int newX = current.point.x + dx[i];
      int newY = current.point.y + dy[i];

      //std::cout << "  start new node (" << newX << ", " << newY << ")" << std::endl;


      // Check if the neighbor is within the grid and not visited
      if (newX >= 0 && newX < rows && newY >= 0 && newY < cols) {

        // std::cout << "  inside new node (" << newX << ", " << newY << ")" << std::endl;

        if (visited[newX][newY])
          {
            // std::cout << "  visited new node (" << newX << ", " << newY << ")" << std::endl;
            continue;
          }

        if (grid[newX][newY] == 1)
          {
            // std::cout << "  corride new node (" << newX << ", " << newY << ")" << std::endl;
            continue;
          }


        // Calculate the cost to move to the neighbor
        int newCost = current.cost + 1; // Assuming uniform cost for simplicity

        // Create the neighbor node
        Node neighbor(Point(newX, newY), newCost, calculateHeuristic(Point(newX, newY), goal));

        // Check if the neighbor is already in the open set
        // auto it = openSet.find(neighbor); // this cannot search the correct node
        auto it = find_if(openSet.begin(), openSet.end(),
                          [&neighbor](Node node)
                          { return (node.point.x == neighbor.point.x && node.point.y == neighbor.point.y);});
        if (it != openSet.end()) {
          // If the neighbor is in the open set, compare costs and update if necessary
          if (neighbor < *it) {
            openSet.erase(it);
          } else {
            continue;
          }
        }

        // std::cout << " add new node (" << newX << ", " << newY << ")" << std::endl;
        openSet.insert(neighbor);
        parent[newX][newY] = current.point;
      }
    }

    cnt++;
  }

  //cout << "Goal not reachable!" << endl;

  return false;
}
};
