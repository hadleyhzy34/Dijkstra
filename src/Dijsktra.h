#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <iostream>
#include <limits.h>
#include <limits>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
/* #include <map> */
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#pragma once

typedef std::vector<std::pair<float, float>> vec_pair;

struct World {
  float minX, maxX;
  float minY, maxY;
  int widthX, widthY;
};

struct Node {
  int idx, idy;
  float px, py;
  float val;
  Node *parent;
  Node(int i, int j, float x, float y, float v, Node *p = nullptr)
      : idx(i), idy(j), px(x), py(y), val(v), parent(p) {
  } // user-defined default constructor
};

class Dijkstra {
private:
  float resolution;
  struct World worldMap;
  float radius;
  Node *startNode;
  Node *endNode;
  std::vector<std::vector<float>> motionVector;
  std::vector<std::vector<bool>> obsMap;

  // openSet and resultSet
  std::unordered_map<int, Node *> openSet;
  std::unordered_map<int, Node *> resultSet;

  // final path
  vec_pair path;

public:
  Dijkstra(float, float, float, float, float, float, struct World);
  ~Dijkstra();
  // build obstacle map
  void buildObsMap(float, float);
  // get position value
  float getPosition(int, float);
  // get index of node
  int getIdx(struct Node *);
  /* Node getNode(float, float); */
  Node *getNode(float, float, float = std::numeric_limits<float>::max());
  /* Node *setNode(float, float, float = std::numeric_limits<float>::max()); */
  // plan path based on dijkstra algorithm
  void plan();

  // check current node boundary and safeness
  bool checkNode(struct Node *);

  // path
  void calcPath();
  vec_pair getPath() { return this->path; }
  void printPath();
};

#endif
