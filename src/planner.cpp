#include "planner.h"

using namespace HybridAStar;

Planner::Planner() {

    // 发布地图
    pubOccupancyMap2 = n.advertise<nav_msgs::OccupancyGrid> ("planning/obstacle/map_inflated", 1);
    downSizeFilter.setLeafSize(Constants::cellSize, Constants::cellSize, Constants::cellSize);
    pc_published = false;
    initializeOccupancyMap();

    // 发布起始点
    pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

    // rviz 交互
    subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);  // 监听终点
    subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);          // 监听起点
};

void Planner::initializeOccupancyMap() {

    occupancyMap2D.header.frame_id = "map";
    // 400x400 cell, 0.05m resolution, 20mx20m size
    occupancyMap2D.info.width = Constants::cellNum;   // 400
    occupancyMap2D.info.height = Constants::cellNum;  // 400
    occupancyMap2D.info.resolution = Constants::cellSize;   // 0.05m
    occupancyMap2D.info.origin.orientation.x = 0.0;
    occupancyMap2D.info.origin.orientation.y = 0.0;
    occupancyMap2D.info.origin.orientation.z = 0.0;
    occupancyMap2D.info.origin.orientation.w = 1.0;
    occupancyMap2D.data.resize(occupancyMap2D.info.width * occupancyMap2D.info.height);
}

void Planner::setMap() {

    if (pc_published) {
        return;
    }

    pcl::PLYReader reader;
    pcl::PointCloud<PointType>::Ptr laserCloudIn(new pcl::PointCloud<PointType>());
    reader.read(std::string(std::string(ROOT_DIR) + "PCD/") + "yinshe_map2.ply", *laserCloudIn);

    // 点云降采样
    pcl::PointCloud<PointType>::Ptr laserCloudInDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setInputCloud(laserCloudIn);
    downSizeFilter.filter(*laserCloudInDS);

    // 过滤地板与天花板
    pcl::PointCloud<PointType>::Ptr laserCloudFiltered(new pcl::PointCloud<PointType>());
    for (int i = 0; i < laserCloudInDS->size(); ++i) {
        PointType p;
        p.x = laserCloudInDS->points[i].x;
        p.y = laserCloudInDS->points[i].y;
        p.z = laserCloudInDS->points[i].z;
        // if (p.z > 2.2 || p.z < 0.2) {
        //     continue;
        // }
        laserCloudFiltered->push_back(p);
    }

    // 更新黑色区域
    if (pubOccupancyMap2.getNumSubscribers() != 0) {

        // 坐标系从地图中心转换到地图左下角
        occupancyMap2D.header.stamp = ros::Time::now();
        occupancyMap2D.info.origin.position.x = 0.0;
        occupancyMap2D.info.origin.position.y = 0.0;
        double halfMapWidth = (Constants::cellNum * Constants::cellSize) / 2.0;  // 20/2m
        double halfMapHeight = (Constants::cellNum * Constants::cellSize) / 2.0;  // 20/2m
        std::fill(occupancyMap2D.data.begin(), occupancyMap2D.data.end(), 0);

        // collisionMap: 点云拓展为球体，半径 (w/2)/0.05 + 1 cell，也可以理解为机器人半径，用于防碰撞
        nav_msgs::OccupancyGrid collisionMap = occupancyMap2D;
        int search_num_ob = ceil(Constants::width/2.0/ Constants::cellSize) + 1;  // (w/2)/0.05 + 1 = 9cell
        for (int i = 0; i < laserCloudFiltered->size(); ++i) {
            int index_x = (laserCloudFiltered->points[i].x + halfMapWidth) / Constants::cellSize;
            int index_y = (laserCloudFiltered->points[i].y + halfMapHeight) / Constants::cellSize;

            for (int m = -search_num_ob; m <= search_num_ob; ++m) {
                for (int n = -search_num_ob; n <= search_num_ob; ++n) {
                    if (sqrt(float(m*m + n*n)) > search_num_ob) {
                        continue;
                    }

                    int x_id = index_x + m;
                    int y_id = index_y + n;
                    if (x_id < 0 || y_id < 0 || x_id >= Constants::cellNum || y_id >= Constants::cellNum) {
                        continue;
                    }
                    // update the black region, taking into account the robot's size.
                    int index = y_id * collisionMap.info.width + x_id;
                    collisionMap.data[index] = 100;
                }
            }

            if (index_x < 0 || index_y < 0 || index_x >= Constants::cellNum || index_y >= Constants::cellNum) {
                continue;
            }
            // update the black region, taking into account the robot's size.
            int index = index_y * occupancyMap2D.info.width + index_x;
            occupancyMap2D.data[index] = 100;
        }
        configurationSpace.updateGrid(collisionMap);
        pubOccupancyMap2.publish(occupancyMap2D);
        pc_published = true;

        // 二值地图
        bool** binMap;
        int height = occupancyMap2D.info.height;  // 400
        int width = occupancyMap2D.info.width;    // 400
        binMap = new bool*[width];
        for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                binMap[x][y] = occupancyMap2D.data[y * width + x]==100 ? true : false;
            }
        }
        // 维诺地图
        voronoiDiagram.initializeMap(width, height, binMap);
        voronoiDiagram.update();
        voronoiDiagram.visualize();  // 生成到 .ros 文件夹下
    }
}

void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {

    float x = initial->pose.pose.position.x / Constants::cellSize;  // 单位：格
    float y = initial->pose.pose.position.y / Constants::cellSize;  // 单位：格
    float t = tf::getYaw(initial->pose.pose.orientation);
    geometry_msgs::PoseStamped startN;
    startN.pose.position = initial->pose.pose.position;
    startN.pose.orientation = initial->pose.pose.orientation;
    startN.header.frame_id = "map";
    startN.header.stamp = ros::Time::now();
    std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

    if (occupancyMap2D.info.height >= y && y >= 0 && occupancyMap2D.info.width >= x && x >= 0) {
        validStart = true;
        start = *initial;
        plan();
        pubStart.publish(startN);
    }
    else {
        std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
    }
}

void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {

    float x = end->pose.position.x / Constants::cellSize;  // 单位：格
    float y = end->pose.position.y / Constants::cellSize;  // 单位：格
    float t = tf::getYaw(end->pose.orientation);
    std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

    if (occupancyMap2D.info.height >= y && y >= 0 && occupancyMap2D.info.width >= x && x >= 0) {
        validGoal = true;
        goal = *end;
        plan();
    }
    else {
        std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
    }
}

void Planner::plan() {

    // if a start as well as goal are defined go ahead and plan
    if (!validStart || !validGoal) {
        std::cout << "missing goal or start" << std::endl;
        return;
    }

    std::cout << "start planning..." << std::endl;
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = occupancyMap2D.info.width;    // 400
    int height = occupancyMap2D.info.height;  // 400
    int depth = Constants::headings;          // 72
    int length = width * height * depth;      // 160000x72
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // 交换 start 和 goal 的位姿。
    // 我们的场景需要在 goal 附近进行微调，从 goal 向 start 搜索更容易。
    // retrieving start position
    float x = goal.pose.position.x / Constants::cellSize;  // 单位：格
    float y = goal.pose.position.y / Constants::cellSize;  // 单位：格
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t + M_PI);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    // retrieving goal position
    x = start.pose.pose.position.x / Constants::cellSize;  // 单位：格
    y = start.pose.pose.position.y / Constants::cellSize;  // 单位：格
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t + M_PI);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);

    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();

    // 核心步骤：
    // 1) 调用 hybridAStar() 函数获取一条路径
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace);
    smoother.tracePath(nSolution);
    path.updatePath(smoother.getPath());
    // 2) 迭代优化，路径平滑
    smoother.smoothPath(voronoiDiagram);
    smoothedPath.updatePath(smoother.getPath());

    std::cout << "finish searching..." << std::endl;

    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();

    delete [] nodes3D;
    delete [] nodes2D;

}
