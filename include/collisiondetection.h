#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar {
namespace {

inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
    x = node->getX();
    y = node->getY();
    // avoid 2D collision checking
    t = 99;
}

inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
    x = node->getX();
    y = node->getY();
    t = node->getT();
}
}

class CollisionDetection {
public:
    // Constructor
    CollisionDetection();

    // 节点是否与环境碰撞
    template<typename T>
    bool isTraversable(const T* node) const {

        float cost = 0;
        float x;
        float y;
        float t;
        // assign values to the configuration
        getConfiguration(node, x, y, t);

        // 2D 碰撞检测不考虑小车大小，直接查询地图
        if (t == 99) {
            return grid.data[node->getIdx()]!=100;
        }
        // 3D 碰撞检测
        else {
            return configurationTest(x, y, t);
        }
    }
    
    /* 3D 碰撞检测*/
    bool configurationTest(float x, float y, float t) const;

    void updateGrid(nav_msgs::OccupancyGrid map) {grid = map;}

private:
    // The occupancy grid
    nav_msgs::OccupancyGrid grid;
    /**
     * 用于 3D 碰撞检测
     * 每个 cell 包含 72*1 个 pose
    **/
    Constants::config collisionLookup[Constants::headings * Constants::positions];
};
}
#endif // COLLISIONDETECTION_H
