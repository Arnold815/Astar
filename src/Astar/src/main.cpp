
/** --Includes--*/
#include <iostream>
#include <vector>
#include <cmath>
#include "../include/Planner.h"
#include "../include/Build_Map.h"

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>


int main(int argc, char** argv) {

  ros::init(argc, argv, "Astar");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);
  
  //std::vector<double> Coordinates;
  std::vector<Vec3i> path;
  //初始化
  double xy_res = 0.25;  //x y的分辨率
  double z_res = 0.25;  // z 的分辨率
  double margin = 0.2;  // 机器人的尺寸

  /** 初始化地图
   * {xmin,ymin,zmin,xmax,ymax,zmax} 
   */
  std::vector<double> Boundary = { 0.0, -5.0, 0.0, 10.0, 20.0, 10.0 };
  // 初始化障碍物
  std::vector<std::vector<double>> Obstacle = {
      { 0.0, 2.0, 0.0, 2.0, 5, 3 },
       { 4, 2.0, 4.5, 5.0, 5.5, 6.0 }, 
       { 4, 4.0, 1.5, 5.0, 7, 3.5 }, 
       { 0.0, 2.0, 4, 3.0, 5.5, 6.5 } };

  // 建立map
  Build_Map Map = Build_Map(Boundary, xy_res, z_res, margin);

  //得到world
  std::vector<int> World = Map.World_Dimensions();

  //实例化 planner类
  Planner Plan = Planner({ World[0], World[1], World[2] });

  //将障碍物添加到 closed列表
  for (const std::vector<double> &v : Obstacle) {
    std::vector<int> Obstacle_Extrema = Map.Build_Obstacle(v);
    for (int Counter_X = Obstacle_Extrema[0]; Counter_X != Obstacle_Extrema[3];
        Counter_X++) {
      for (int Counter_Y = Obstacle_Extrema[1];
          Counter_Y != Obstacle_Extrema[4]; Counter_Y++) {
        for (int Counter_Z = Obstacle_Extrema[2];
            Counter_Z != Obstacle_Extrema[5]; Counter_Z++) {
          Plan.Add_Collision({ Counter_X, Counter_Y, Counter_Z });
        }
      }
    }
  }

  // 计算距离设置为欧吉里的 或者 曼哈顿
  Plan.Set_Heuristic(Planner::Manhattan);
  std::cout << "Calculating Shortest Path ... \n";
  std::vector<double> Start = { 0, 0.5, 8 };  //起点
  std::vector<double> Goal = { 6, 6.4, 0 };  //终点

  // 检查起点 终点 是否合法
  if ((Start[0] < Boundary[0] || Start[0] > Boundary[3])
      || (Start[1] < Boundary[1] || Start[1] > Boundary[4])
      || (Start[2] < Boundary[2] || Start[2] > Boundary[5])) {
    std::cout << "Start Point Lies Out of Workspace.";
  } else if ((Goal[0] < Boundary[0] || Goal[0] > Boundary[3])
      || (Goal[1] < Boundary[1] || Goal[1] > Boundary[4])
      || (Goal[2] < Boundary[2] || Goal[2] > Boundary[5])) {
    std::cout << "Goal Point Lies Out of Workspace.";
  } else {
    
    std::vector<int> Start_Node = Map.Build_Node(Start);
    std::vector<int> Goal_Node = Map.Build_Node(Goal);

    //auto path = Plan.findPath({ Start_Node[0], Start_Node[1], Start_Node[2] },
    //                         { Goal_Node[0], Goal_Node[1], Goal_Node[2] });
    path = Plan.findPath({ Start_Node[0], Start_Node[1], Start_Node[2] },
                              { Goal_Node[0], Goal_Node[1], Goal_Node[2] });

    // 打印路径
    std::cout << "X\tY\tZ\n";
    for (auto& coordinate : path) {
      std::vector<int> Discrete_Node = { coordinate.x, coordinate.y, coordinate
          .z };
      std::vector<double> Coordinates = Map.Get_Coordinate(Discrete_Node);
      //Coordinates = Map.Get_Coordinate(Discrete_Node);
      std::cout << Coordinates[0] << "\t" << Coordinates[1] << "\t"
          << Coordinates[2] << "\n";
    }
  }



  while (ros::ok()) //可视化
  {
    
    visualization_msgs::Marker pathline, start_point, goal_point;
    pathline.header.frame_id = start_point.header.frame_id = goal_point.header.frame_id  = "/my_frame";
    pathline.header.stamp = start_point.header.stamp = goal_point.header.stamp =  ros::Time::now();
    pathline.ns = start_point.ns = goal_point.ns = "Astar";
    pathline.action = start_point.action = goal_point.action = visualization_msgs::Marker::ADD;
    pathline.pose.orientation.w = start_point.pose.orientation.w = goal_point.pose.orientation.w = 1.0;

    std::vector<visualization_msgs::Marker> obstacle_cube(Obstacle.size());
    int i = 0;
    for(auto& _obstacle : obstacle_cube)
    {
      _obstacle.header.frame_id = "/my_frame";
      _obstacle.header.stamp =  ros::Time::now();
      _obstacle.ns = "Astar";
      _obstacle.action = visualization_msgs::Marker::ADD;
      _obstacle.pose.orientation.w = 1.0;
      _obstacle.id = i+3;
      _obstacle.type = goal_point.type = visualization_msgs::Marker::CUBE;
      _obstacle.color.r = 0.5;
      _obstacle.color.g = 0.5;
      _obstacle.color.a = 1.0;

      _obstacle.pose.position.x = (Obstacle[i][0] + Obstacle[i][3])/2;
      _obstacle.pose.position.y = (Obstacle[i][1] + Obstacle[i][4])/2;
      _obstacle.pose.position.z = (Obstacle[i][2] + Obstacle[i][5])/2;
      _obstacle.scale.x = abs(Obstacle[i][0] - Obstacle[i][3]);
      _obstacle.scale.y = abs(Obstacle[i][1] - Obstacle[i][4]);
      _obstacle.scale.z = abs(Obstacle[i][2] - Obstacle[i][5]);
      marker_pub.publish(_obstacle);
      i++;
    }
   

    pathline.id = 0;
    start_point.id = 1;
    goal_point.id = 2;


    //type
    pathline.type = visualization_msgs::Marker::LINE_STRIP;
    start_point.type = visualization_msgs::Marker::POINTS;
    goal_point.type = visualization_msgs::Marker::POINTS;

    //scale 
    start_point.scale.x = 0.2;
    start_point.scale.y = 0.2;
    
    goal_point.scale.x = 0.2;
    goal_point.scale.y = 0.2;

    pathline.scale.x = 0.1;
    //
    
    // green
    pathline.color.g = 1.0f;
    pathline.color.a = 1.0;

     //  blue
    start_point.color.b = 1.0;
    start_point.color.a = 1.0;

    // red
    goal_point.color.r = 1.0;
    goal_point.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = Start[0];
    p.y = Start[1];
    p.z = Start[2];
    start_point.points.push_back(p);

    p.x = Goal[0];
    p.y = Goal[1];
    p.z = Goal[2];
    goal_point.points.push_back(p);
    // 发布路径点
    for (auto& coordinate : path) {
      std::vector<int> Discrete_Node = { coordinate.x, coordinate.y, coordinate
          .z };
      std::vector<double> Coordinates = Map.Get_Coordinate(Discrete_Node);
      geometry_msgs::Point p;
      p.x = Coordinates[0];
      p.y = Coordinates[1];
      p.z = Coordinates[2];

      pathline.points.push_back(p);
    }

    marker_pub.publish(pathline);
    marker_pub.publish(start_point);
    marker_pub.publish(goal_point);

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
   
    r.sleep();

  }
  return 0;  
}
