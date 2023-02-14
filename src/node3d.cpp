#include "node3d.h"

using namespace HybridAStar;


const int Node3D::dir = 3;
// 此处需要根据最小弯转半径作修改。r=2m
const float Node3D::dy[] = {0,   -1.1945,  1.1945};  // 单位：格
const float Node3D::dx[] = {10.0, 9.7023,  9.7023};  // 单位：格
const float Node3D::dt[] = {0,    0.2450, -0.2450};


bool Node3D::isOnGrid(const int width, const int height) const {
    return x >= 0 && x < width && y >= 0 && y < height && (int)(t / Constants::deltaHeadingRad) >= 0 && (int)(t / Constants::deltaHeadingRad) < Constants::headings;
}

bool Node3D::isInRange(const Node3D& goal) const {
    int random = rand() % 10 + 1;
    float dx = std::abs(x - goal.x) / random;
    float dy = std::abs(y - goal.y) / random;
    return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;
}

Node3D* Node3D::createSuccessor(const int i) {
    float xSucc;
    float ySucc;
    float tSucc;

    // calculate successor positions forward
    if (i < 3) {
        xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
        ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
        tSucc = Helper::normalizeHeadingRad(t + dt[i]);
    }
    // backwards
    else {
        xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
        ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
        tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
    }

    return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}

void Node3D::updateG() {
    // forward driving
    if (prim < 3) {
        // penalize turning
        if (pred->prim != prim) {
            // penalize change of direction
            if (pred->prim > 2) {
                g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
            } else {
                g += dx[0] * Constants::penaltyTurning;
            }
        }
        else {
            g += dx[0];
        }
    }
    // reverse driving
    else {
        // penalize turning and reversing
        if (pred->prim != prim) {
            // penalize change of direction
            if (pred->prim < 3) {
                g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
            }
            else {
                g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
            }
        }
        else {
            g += dx[0] * Constants::penaltyReversing;
        }
    }
}

bool Node3D::operator == (const Node3D& rhs) const {
    return (int)x == (int)rhs.x &&
           (int)y == (int)rhs.y &&
           (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
            std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
