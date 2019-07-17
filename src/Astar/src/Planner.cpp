

/* --Includes-- */
#include <iostream>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include <set>
#include "../include/Planner.h"

/**
 * 将坐标转化为node的构造函数，初始化 G=H=0
 */
Node::Node(Vec3i coordinates_, Node *Parent_)
    : coordinates(coordinates_) {
  Parent = Parent_;
  G = H = 0;
}

/**
 * 计算 F = G + H
 */
double Node::Get_Score() {
  return G + H;
}


bool Vec3i::operator ==(const Vec3i& coordinates_) {
  return (x == coordinates_.x && y == coordinates_.y && z == coordinates_.z);
}


Vec3i operator +(const Vec3i& left_, const Vec3i& right_) {
  return {left_.x + right_.x, left_.y + right_.y, left_.z + right_.z};
}


/**
 * planner类的构造函数，
 * 初始化 启发模式为欧几里得距离
 * 初始化 方向
 * 初始化 地
 */
Planner::Planner(Vec3i World_Size_) { 
  Set_Heuristic(&Planner::Euclidean);  ///设置默认模式为欧几里得距离
  direction = {
    { 0 , 0 , 1}, {0 , 1, 0}, {1, 0, 0}, {0, 0, -1},
    { 0, -1, 0}, {-1, 0, 0}, {0 , 1, 1}, {1, 0, 1},
    { 1, 1, 0}, {0, -1, -1}, {-1, 0, -1}, {-1, -1, 0},
    { 0, 1, -1}, {0, -1, 1}, {1, 0, -1}, {-1, 0, 1},
    { 1, -1, 0}, {-1, 1, 0}, {1, 1, -1}, {1, -1, 1},
    { -1, 1, 1}, {1, -1, -1}, {-1, -1, 1}, {-1, 1, -1},
    { 1, 1, 1}, {-1, -1, -1}
  };
  World_Size = World_Size_;
}

/**
 * 设置邻居搜索的启发模式
 */
void Planner::Set_Heuristic(std::function<double(Vec3i, Vec3i)> heuristic_) {
  heuristic = std::bind(heuristic_, std::placeholders::_1,
                        std::placeholders::_2);
}

/**
 * 增加点到close列表
 */
void Planner::Add_Collision(Vec3i coordinates_) {
  walls.push_back(coordinates_);
}

/**
 * 寻找路径
 */
std::vector<Vec3i> Planner::findPath(Vec3i Start_, Vec3i Goal_) {
  Node *current = nullptr;  // 设置当前节点为空
  std::set<Node*> Open_Set, Closed_Set;  // 初始化 open 和 closed 列表
  Open_Set.insert(new Node(Start_));  // 将开始位置节点 插入到open列表

  while (true) {
    
    current = *Open_Set.begin();

    //搜索最小F 为current
    for (auto node : Open_Set) {
      if (node->Get_Score() <= current->Get_Score()) {
        current = node;
      }
    }
    // 到达目标
    if (current->coordinates == Goal_) {
      break;
    }

    Closed_Set.insert(current);
    Open_Set.erase(std::find(Open_Set.begin(), Open_Set.end(), current));
    //从所有可能的方向，检查临近点
    for (double i = 0; i < 26; ++i) {
      Vec3i newCoordinates(current->coordinates + direction[i]);
      // 判断是否为障碍
      if (Detect_Collision(newCoordinates)
          || Find_Node(Closed_Set, newCoordinates)) {
        continue;
      }
      // 计算临近点的F值
      double Total_Cost = current->G
          + ((i < 6) ? 100 : ((i > 5 && i < 18) ? 141 : 173));
      Node *successor = Find_Node(Open_Set, newCoordinates);
      if (successor == nullptr) {
        successor = new Node(newCoordinates, current);
        successor->G = Total_Cost;
        successor->H = heuristic(successor->coordinates, Goal_);
        Open_Set.insert(successor);
      } else if (Total_Cost < successor->G) { //如果相邻点在open中，只取G相对较低的
        
        successor->Parent = current;
        successor->G = Total_Cost;
      }
    }

    if (Open_Set.empty()) {
      std::cout << "Path Not Found";
      break;
    }
  }
  //储存路径
  std::vector<Vec3i> path;
  while (current != nullptr) {
    path.push_back(current->coordinates);
    current = current->Parent;
  }
  return path;  //返回路径
}

/**
 * 在节点列表中找到给点坐标的节点
 */
Node* Planner::Find_Node(std::set<Node*>& nodes_, Vec3i coordinates_) {
  for (auto node : nodes_) {
    if (node->coordinates == coordinates_) {
      return node;  
    }
  }
  return nullptr; 
}

/**
 * 检测是否位于障碍物内 或者 超出边界
 */
bool Planner::Detect_Collision(Vec3i coordinates_) {
  if (coordinates_.x < 0 || coordinates_.x >= World_Size.x || coordinates_.y < 0
      || coordinates_.y >= World_Size.y || coordinates_.z < 0
      || coordinates_.z >= World_Size.z
      || std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
    return true;  
  }
  return false; 
}

/**
 * 给出两点之间的绝对距离
 */
Vec3i Planner::Distance(Vec3i Now_, Vec3i Neighbor_) {
  return {abs(Now_.x - Neighbor_.x), abs(Now_.y - Neighbor_.y),
    abs(Now_.z - Neighbor_.z)};
}

/**
 * 计算两点之间的欧几里得距离
 */
double Planner::Euclidean(Vec3i Now_, Vec3i Neighbor_) {
  auto delta = std::move(Distance(Now_, Neighbor_));
  return static_cast<double>(100
      * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
}

/**
 * 计算两点之间的曼哈顿距离
 */
double Planner::Manhattan(Vec3i Now_, Vec3i Neighbor_) {
  auto delta = std::move(Distance(Now_, Neighbor_));
  return static_cast<double>(100 * (delta.x + delta.y + delta.z));
}

/**
 *  Planner Class 的析构函数
 * */
Planner::~Planner() {
}
