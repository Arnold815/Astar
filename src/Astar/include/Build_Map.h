
#ifndef MID_TERM_A_STAR_3D_INCLUDE_BUILD_MAP_H_
#define MID_TERM_A_STAR_3D_INCLUDE_BUILD_MAP_H_

/* --Includes-- */
#include <vector>
#include <functional>
#include <set>
#include "Planner.h"




class Build_Map {
 public:  

  Build_Map(std::vector<double>, double, double, double);  
  std::vector<double> Boundary;
  std::vector<int> World;
  double xy_res, z_res, margin;
 
  std::vector<int> World_Dimensions();
  std::vector<int> Build_Obstacle(std::vector<double>);
  std::vector<int> Build_Node(std::vector<double>);
  std::vector<double> Get_Coordinate(std::vector<int>);

  std::vector<std::vector<std::vector<int>>> Build_costmap(std::vector<std::vector<double>> v);//构建膨胀层
  virtual ~Build_Map(); 
};

#endif  
