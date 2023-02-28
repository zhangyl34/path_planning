#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);
// 计算到目标的启发值 (cost): max(Reed-Shepp距离/Dubins距离, 欧氏距离)
// 1) "non-holonomic-without-obstacles"
// 2) "holonomic-with-obstacles": A*
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace);
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);

// 重载运算符，用来生成节点的比较
struct CompareNodes {
    // Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const Node3D* lhs, const Node3D* rhs) const {
        return lhs->getC() > rhs->getC();
    }
    // Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const Node2D* lhs, const Node2D* rhs) const {
        return lhs->getC() > rhs->getC();
    }
};

Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace) {

    // PREDECESSOR AND SUCCESSOR INDEX
    int iPred, iSucc;
    float newG;
    // Number of possible directions, 3 for forward driving and an additional 3 for reversing
    int dir = Constants::reverse ? 6 : 3;
    // Number of iterations the algorithm has run for stopping based on Constants::iterations
    int iterations = 0;

    // OPEN LIST AS BOOST IMPLEMENTATION
    typedef boost::heap::binomial_heap<Node3D*,boost::heap::compare<CompareNodes>> priorityQueue;
    priorityQueue O;

    // 计算到目标的启发式值
    updateH(start, goal, nodes2D, width, height, configurationSpace);
    // mark start as open
    start.open();
    // push on priority queue aka open list
    O.push(&start);
    // 计算索引
    iPred = start.setIdx(width, height);
    nodes3D[iPred] = start;

    // NODE POINTER
    Node3D* nPred;
    Node3D* nSucc;

    // continue until O empty
    while (!O.empty()) {

        // pop node with lowest cost from priority queue
        nPred = O.top();
        // set index
        iPred = nPred->setIdx(width, height);
        iterations++;

        // LAZY DELETION of rewired node
        // if there exists a pointer this node has already been expanded
        if (nodes3D[iPred].isClosed()) {  // 如果为closed，说明该点已经处理过
            // pop node from the open list and start with a fresh node
            O.pop();
            continue;
        }
        else if (!nodes3D[iPred].isOpen()) {
            continue;
        }

        // add node to closed list
        nodes3D[iPred].close();  // 将它的状态标记为closed
        // remove node from open list
        O.pop();

        // 抵达目标则退出循环
        if (*nPred == goal || iterations > Constants::iterations) {
            if (iterations > Constants::iterations) {
                std::cout << "can't find a suitable path." << std::endl;
            }
            return nPred;
        }
        // 继续搜索
        // 当前位置靠近 goal 时，考虑用 dubinsShot 去命中目标点
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
            nSucc = dubinsShot(*nPred, goal, configurationSpace);

            // 如果 Dubins 方法能直接命中，即不需要进入 Hybrid A* 搜索了，直接返回结果
            if (nSucc != nullptr && *nSucc == goal) {
                return nSucc;
            }
        }

        for (int i = 0; i < dir; i++) {
            // 创建下一个扩展节点，这里有三种可能的方向，如果可以倒车的话是六种方向
            nSucc = nPred->createSuccessor(i);
            // set index of the successor
            iSucc = nSucc->setIdx(width, height);

            // ensure successor is on grid ROW MAJOR
            // ensure successor is not blocked by obstacle
            if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {

                // ensure successor is not on closed list
                // or the successor is in the same cell
                if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

                    // 已走过的长度
                    nSucc->updateG();
                    newG = nSucc->getG();

                    // if successor not on open list or found a shorter way to the cell
                    // or the successor is in the same cell
                    if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                        // 预计到终点的路程
                        updateH(*nSucc, goal, nodes2D, width, height, configurationSpace);

                        // if the successor is in the same cell but the Cost (g+h) is larger
                        if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                            delete nSucc;
                            continue;
                        }
                        // if successor is in the same cell and the Cost (g+h) is lower, set predecessor to predecessor of predecessor
                        else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                            nSucc->setPred(nPred->getPred());
                        }

                        if (nSucc->getPred() == nSucc) {
                            std::cout << "error? looping";
                        }

                        // put successor on open list
                        nSucc->open();
                        nodes3D[iSucc] = *nSucc;
                        O.push(&nodes3D[iSucc]);
                        delete nSucc;
                    } else { delete nSucc; }
                } else { delete nSucc; }
            } else { delete nSucc; }
        }
    }

    if (O.empty()) {
        return nullptr;
    }

    return nullptr;
}

float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,   // 400
            int height,  // 400
            CollisionDetection& configurationSpace) {

    // PREDECESSOR AND SUCCESSOR INDEX
    int iPred, iSucc;
    float newG;

    // reset the open and closed list
    for (int i = 0; i < width * height; ++i) {
        nodes2D[i].reset();
    }

    boost::heap::binomial_heap<Node2D*,boost::heap::compare<CompareNodes>> O;
    // h = start 到 goal 的直线距离
    start.updateH(goal);
    // mark start as open
    start.open();
    // push on priority queue
    O.push(&start);
    iPred = start.setIdx(width);  // set and get the index.
    nodes2D[iPred] = start;

    // NODE POINTER
    Node2D* nPred;
    Node2D* nSucc;

    std::cout << "start while iteration" << std::endl;
    // continue until O empty
    while (!O.empty()) {
        // pop node with lowest cost from priority queue
        nPred = O.top();
        // set and get the index
        iPred = nPred->setIdx(width);

        // LAZY DELETION of rewired node
        // if there exists a pointer this node has already been expanded
        if (nodes2D[iPred].isClosed()) {
            // pop node from the open list and start with a fresh node
            O.pop();
            continue;
        }
        else if (nodes2D[iPred].isOpen()) {
            // add node to closed list
            nodes2D[iPred].close();
            nodes2D[iPred].discover();

            // remove node from open list
            O.pop();

            // 判断是否抵达目标
            if (*nPred == goal) {
                return nPred->getG();
            }
            // 继续搜索
            else {
                for (int i = 0; i < Node2D::dir; i++) {
                    // 朝 dir 扩展一个节点
                    nSucc = nPred->createSuccessor(i);
                    // set and get the index
                    iSucc = nSucc->setIdx(width);

                    // ensure successor is on grid ROW MAJOR
                    // ensure successor is not blocked by obstacle
                    // ensure successor is not on closed list
                    if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)
                        && !nodes2D[iSucc].isClosed()) {

                        // 已走过的长度
                        nSucc->updateG();
                        newG = nSucc->getG();

                        // if successor not on open list or g value lower than before put it on open list
                        if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
                            // 到终点的距离
                            nSucc->updateH(goal);
                            // put successor on open list
                            nSucc->open();
                            nodes2D[iSucc] = *nSucc;
                            O.push(&nodes2D[iSucc]);
                            delete nSucc;
                        }
                        else { delete nSucc; }
                    }
                    else { delete nSucc; }
                }
            }
        }
    }

    // return large number to guide search away
    return 1000;
}

void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, int width, int height,
    CollisionDetection& configurationSpace) {
  
    float dubinsCost = 0;
    float reedsSheppCost = 0;
    float twoDCost = 0;
    float twoDoffset = 0;

    // 使用 dubins 曲线
    if (Constants::dubins) {
        ompl::base::DubinsStateSpace dubinsPath(Constants::r);
        State* dbStart = (State*)dubinsPath.allocState();
        State* dbEnd = (State*)dubinsPath.allocState();
        dbStart->setXY(start.getX(), start.getY());
        dbStart->setYaw(start.getT());
        dbEnd->setXY(goal.getX(), goal.getY());
        dbEnd->setYaw(goal.getT());
        dubinsCost = dubinsPath.distance(dbStart, dbEnd);
    }
    // 或者使用 reedsshepp 曲线
    if (Constants::reverse && !Constants::dubins) {
        ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
        State* rsStart = (State*)reedsSheppPath.allocState();
        State* rsEnd = (State*)reedsSheppPath.allocState();
        rsStart->setXY(start.getX(), start.getY());
        rsStart->setYaw(start.getT());
        rsEnd->setXY(goal.getX(), goal.getY());
        rsEnd->setYaw(goal.getT());
        reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);  // 单位：格
    }

    // 使用 aStar 搜索
    if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
        Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
        Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
        nodes2D[(int)start.getY() * width + (int)start.getX()].setG(
            aStar(goal2d, start2d, nodes2D, width, height, configurationSpace));
    }
    // 计算启发式
    if (Constants::twoD) {
        twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) *
                          ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                          ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) *
                          ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
        twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
    }

    // 更新 h
    start.setH(std::max(twoDCost, std::max(dubinsCost, reedsSheppCost)));
    return;
}

Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  
    // start
    double q0[] = { start.getX(), start.getY(), start.getT() };
    // goal
    double q1[] = { goal.getX(), goal.getY(), goal.getT() };
    // initialize the path
    DubinsPath path;
    // calculate the path
    dubins_init(q0, q1, Constants::r, &path);
    float length = dubins_path_length(&path);  // 单位：格

    Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

    // avoid duplicate waypoint
    int i = 0;
    float x = 0.f;
    x += Constants::dubinsStepSize;  // 10 格
    while (x < length) {
        double q[3];
        dubins_path_sample(&path, x, q);
        dubinsNodes[i].setX(q[0]);  // 单位：格
        dubinsNodes[i].setY(q[1]);
        dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

        // collision check
        if (configurationSpace.isTraversable(&dubinsNodes[i])) {
            // set the predecessor to the previous step
            if (i > 0) {
                dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
            }
            else {
                dubinsNodes[i].setPred(&start);
            }

            if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
                std::cout << "looping shot";
            }

            x += Constants::dubinsStepSize;
            i++;
        }
        else {
            // delete all nodes
            delete [] dubinsNodes;
            return nullptr;
        }
    }

    return &dubinsNodes[i - 1];
}
