#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/type_traits.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include "constants.h"
#include "helper.h"
#include "collisiondetection.h"
#include "dynamicvoronoi.h"
#include "algorithm.h"
#include "node3d.h"
#include "path.h"
#include "smoother.h"

namespace HybridAStar {

typedef pcl::PointXYZ PointType;

class Planner {

public:

    Planner();

    void setMap();
    void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
    void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

    void plan();

private:

    // 初始化 occupancy map
    void initializeOccupancyMap();

    // The node handle
    ros::NodeHandle n;

    // The path produced by the hybrid A* algorithm
    Path path;
    // The smoother used for optimizing the path
    Smoother smoother;
    // The path smoothed
    Path smoothedPath = Path(true);
    // The collission detection for testing specific configurations
    CollisionDetection configurationSpace;
    // The voronoi diagram
    DynamicVoronoi voronoiDiagram;

    /* 与 Rviz 交互*/
    // 地图
    ros::Publisher pubOccupancyMap2;       // 发布 black and grey region
    pcl::VoxelGrid<PointType> downSizeFilter;
    bool pc_published;
    nav_msgs::OccupancyGrid occupancyMap2D;
    // 开始位姿
    ros::Subscriber subStart;
    geometry_msgs::PoseWithCovarianceStamped start;
    bool validStart = false;
    ros::Publisher pubStart;
    // 目标位姿
    ros::Subscriber subGoal;
    geometry_msgs::PoseStamped goal;
    bool validGoal = false;

};
}
#endif // PLANNER_H
