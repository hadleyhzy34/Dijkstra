#include "Dijsktra.h"
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <math.h>
#include <unordered_map>
#include <utility>
/* std::ostream &operator<<(std::ostream &os, */
/*                          const std::unordered_map<int, struct Node> &m) { */
/*   for (auto x : m) { */
/*     os << x.first << " " << x.second.x << "||" << x.second.y << "||" */
/*        << x.second.xId << "||" << x.second.yId << "||" << x.second.cost; */
/*   } */
/*   os << std::endl; */
/*   return os; */
/* } */

Dijkstra::Dijkstra(float ox, float oy, float gx, float gy, float res, float rad,
                   struct World map)
    : resolution(res), radius(rad), worldMap(map) {
  this->motionVector = {
      {1, 0, 1},         {0, 1, 1},          {-1, 0, 1},
      {0, -1, 1},        {-1, -1, sqrtf(2)}, {-1, 1, sqrtf(2)},
      {1, -1, sqrtf(2)}, {1, 1, sqrtf(2)},
  };

  this->startNode = this->getNode(ox, oy, 0.0);
  this->endNode = this->getNode(gx, gy);

  this->worldMap.widthX =
      (this->worldMap.maxX - this->worldMap.minX) / this->resolution;
  this->worldMap.widthY =
      (this->worldMap.maxY - this->worldMap.minY) / this->resolution;

  this->obsMap = std::vector<std::vector<bool>>(
      this->worldMap.widthY, std::vector<bool>(this->worldMap.widthX, false));
}

Dijkstra::~Dijkstra() {}

Node Dijkstra::getNode(float px, float py, float val) {
  int idx = std::round((px - this->worldMap.minX) / this->resolution);
  int idy = std::round((py - this->worldMap.minY) / this->resolution);
  return {idx, idy, px, py, val, -1};
}

float Dijkstra::getPosition(int index, float minPosition) {
  return index * this->resolution + minPosition;
}

int Dijkstra::getIdx(Node node) {
  return node.idy * this->worldMap.widthX + node.idx;
}

int Dijkstra::getIdx(float px, float py) {
  int idx = std::round((px - this->worldMap.minX) / this->resolution);
  int idy = std::round((py - this->worldMap.minY) / this->resolution);

  return idy * this->worldMap.widthX + idx;
}

void Dijkstra::calcObsMap(float ox, float oy) {
  for (auto i = 0; i < this->worldMap.widthX; i++) {
    float x = i * this->resolution + this->worldMap.minX;
    for (auto j = 0; j < this->worldMap.widthY; j++) {
      float y = j * this->resolution + this->worldMap.minY;
      float d = std::pow(x - ox, 2) + std::pow(y - oy, 2);
      if (d < std::pow(this->radius, 2)) {
        this->obsMap[i][j] = true;
      }
    }
  }
}

bool Dijkstra::checkNode(struct Node node) {
  if (node.px < this->worldMap.minX) {
    return false;
  } else if (node.py < this->worldMap.minY) {
    return false;
  } else if (node.px >= this->worldMap.maxX) {
    return false;
  } else if (node.py >= this->worldMap.maxY) {
    return false;
  }

  if (this->obsMap[node.idx][node.idy]) {
    return false;
  }

  return true;
}

void Dijkstra::plan() {
  this->resultSet[this->getIdx(this->startNode)] = this->startNode;
  this->openSet[this->getIdx(this->endNode)] = this->endNode;

  while (true) {
    if (this->openSet.empty()) {
      std::cout << "open set is empty\n";
      break;
    }

    // dfs on all neighbors of resultSet
    for (auto const &res : this->resultSet) {
      Node node = res.second;
      // get current node neighbors
      for (auto motion : this->motionVector) {
        float nPx = node.px + motion[0] * this->resolution;
        float nPy = node.py + motion[1] * this->resolution;
        int nIdx = this->getIdx(nPx, nPy);

        // check node if its already in resultSet
        if (this->resultSet.find(nIdx) != this->resultSet.end()) {
          continue;
        }

        // check node if its already in openSet
        if (this->openSet.find(nIdx) == this->openSet.end()) {
          Node neighbor = this->getNode(nPx, nPy);

          if (!this->checkNode(neighbor)) {
            continue;
          }

          // add this node to openset
          this->openSet[nIdx] = neighbor;
        }

        // update this node shortest path value
        if (motion[2] * this->resolution + node.val < this->openSet[nIdx].val) {
          this->openSet[nIdx].val = motion[2] * this->resolution + node.val;
          this->openSet[nIdx].parent = res.first;
        }
      }
    }

    auto iter = std::min_element(openSet.begin(), openSet.end(),
                                 [](const auto &l, const auto &r) {
                                   return l.second.val < r.second.val;
                                 });

    int curIdx = iter->first;
    Node curNode = iter->second;

    // check if this node is reached, be careful that curNode and endNode
    // are two different node object
    if (curNode.idx == this->endNode.idx && curNode.idy == this->endNode.idy) {
      endNode.parent = curNode.parent;
      std::cout << curNode.idx << "||" << curNode.idy << "||" << curNode.parent
                << std::endl;
      std::cout << "goal found\n";
      break;
    }
    // mv this node from openSet to resultSet
    this->openSet.erase(curIdx);
    this->resultSet[curIdx] = curNode;
    /* std::cout << "check existence: " << curNode.val << std::endl; */
  }
}

void Dijkstra::calcPath() {
  Node curNode = this->endNode;
  this->path.push_back(std::make_pair(curNode.px, curNode.py));

  std::cout << "node position is: " << curNode.idx << "||" << curNode.idy
            << "||" << curNode.px << " " << curNode.py << " " << curNode.parent
            << std::endl;

  while (curNode.parent != -1) {
    curNode = this->resultSet[curNode.parent];
    this->path.push_back(std::make_pair(curNode.px, curNode.py));
    std::cout << "node position is: " << curNode.px << " " << curNode.py
              << std::endl;
  }
}

void Dijkstra::printPath() {
  std::cout << "path length: " << this->path.size() << std::endl;
  if (this->path.empty()) {
    return;
  }

  for (auto p : this->path) {
    std::cout << p.first << " " << p.second << std::endl;
  }
}
