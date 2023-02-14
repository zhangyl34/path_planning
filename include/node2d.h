#ifndef NODE2D_H
#define NODE2D_H

#include <cmath>

#include "constants.h"
namespace HybridAStar {

/* 用于 2D A* 搜索*/
class Node2D {
public:

    Node2D(): Node2D(0, 0, 0, 0, nullptr) {}
    Node2D(int x, int y, float g, float h, Node2D* pred) {
        this->x = x;
        this->y = y;
        this->g = g;
        this->h = h;
        this->pred = pred;
        this->o = false;
        this->c = false;
        this->d = false;
        this->idx = -1;
    }

    int getX() const { return x; }
    int getY() const { return y; }
    float getG() const { return g; }
    float getH() const { return h; }
    // c = g+h
    float getC() const { return g + h; }
    int getIdx() const { return idx; }
    bool  isOpen() const { return o; }
    bool  isClosed() const { return c; }
    bool  isDiscovered() const { return d; }
    // get a pointer to the predecessor
    Node2D* getPred() const { return pred; }

    void setX(const int& x) { this->x = x; }
    void setY(const int& y) { this->y = y; }
    void setG(const float& g) { this->g = g; }
    void setH(const float& h) { this->h = h; }
    // set and get the index of the node in the 2D array
    int setIdx(int width) { this->idx = y * width + x; return idx;}
    // open the node
    void open() { o = true; c = false; }
    // close the node
    void close() { c = true; o = false; }
    // set the node neither open nor closed
    void reset() { c = false; o = false; }
    // discover the node
    void discover() { d = true; }
    // set a pointer to the predecessor of the node
    void setPred(Node2D* pred) { this->pred = pred; }

    // Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
    void updateG() { g += movementCost(*pred); d = true; }
    // Updates the cost-to-go for the node x' to the goal node.
    void updateH(const Node2D& goal) { h = movementCost(goal); }
    // The heuristic as well as the cost measure.
    float movementCost(const Node2D& pred) const { return sqrt((x - pred.x) * (x - pred.x) + (y - pred.y) * (y - pred.y)); }

    // Custom operator to compare nodes. Nodes are equal if their x and y position is the same.
    bool operator == (const Node2D& rhs) const { return x == rhs.x && y == rhs.y; }

    // Validity check to test, whether the node is in the 2D array.
    bool isOnGrid(const int width, const int height) const { return  x >= 0 && x < width && y >= 0 && y < height; }

    // Creates a successor on a eight-connected grid.
    Node2D* createSuccessor(const int i);

    // CONSTANT VALUES
    // Number of possible directions
    static const int dir;
    // Possible movements in the x direction
    static const int dx[];
    // Possible movements in the y direction
    static const int dy[];

private:
    // the x position
    int x;
    // the y position
    int y;
    // the cost-so-far
    float g;
    // the cost-to-go
    float h;
    // the index of the node in the 2D array
    int idx;
    // the open value
    bool o;
    // the closed value
    bool c;
    // the discovered value
    bool d;
    // the predecessor pointer
    Node2D* pred;
};
}
#endif // NODE2D_H
