#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "node3d.h"
#include "node2d.h"
#include "collisiondetection.h"

namespace HybridAStar {
class Node3D;
class Node2D;
class Visualize;

class Algorithm {

public:
    // The deault constructor
    Algorithm() {}

    /********
    输入：
        始点与目标点、
        配置空间的3维和2维表示（2D用来A*，3D用于hybrid A*）、
        搜索网格的宽度及高度、
        配置空间的查找表、
        RVIZ可视化类(用于显示结果)
    返回：
        满足约束条件的节点（数据结构用指针表示）
    ********/
    static Node3D* hybridAStar(Node3D& start,
                                const Node3D& goal,
                                Node3D* nodes3D,
                                Node2D* nodes2D,
                                int width,
                                int height,
                                CollisionDetection& configurationSpace);

};
}
#endif // ALGORITHM_H
