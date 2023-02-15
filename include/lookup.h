#ifndef COLLISIONLOOKUP
#define COLLISIONLOOKUP

#include "dubins.h"
#include "constants.h"

namespace HybridAStar {
namespace Lookup {

inline int sign(double x) {
    if (x >= 0) { return 1; }
    else { return -1; }
}

/**
 * 输入的 lookup[] 大小为 72*1
**/
inline void collisionLookup(Constants::config* lookup) {

    bool DEBUG = false;
    const float cSize = Constants::cellSize;  // 0.05m
    const int size = Constants::bbSize;       // 31 格

    struct point {
        double x;
        double y;
    };

    // 矩形中心
    point c;
    point temp;
    // 矩形角点
    point p[4];
    point nP[4];
    // 朝向
    double theta;

    // 网格遍历变量
    point t;
    point start;
    point end;
    // cell index
    int X;
    int Y;
    // t value for crossing vertical and horizontal boundary
    double tMaxX;
    double tMaxY;
    // t value for width/heigth of cell
    double tDeltaX;
    double tDeltaY;
    // positive or negative step direction
    int stepX;
    int stepY;
    // grid
    bool cSpace[size * size];  // bbox 最多占的格数
    bool inside = false;
    int hcross1 = 0;
    int hcross2 = 0;

    // cell 的分辨率
    const int positionResolution = Constants::positionResolution;  // 1
    const int positions = Constants::positions;  // 1x1
    // cell 内的 points
    point points[positions];
    // 计算出 points 的真实偏移量 [0-1]
    for (int i = 0; i < positionResolution; ++i) {
        for (int j = 0; j < positionResolution; ++j) {
            points[positionResolution * i + j].x = 1.f / positionResolution * j;
            points[positionResolution * i + j].y = 1.f / positionResolution * i;
        }
    }

    for (int q = 0; q < positions; ++q) {  // 1x1
        // set the starting angle to zero;
        theta = 0;

        // set points of rectangle
        c.x = (double)size / 2 + points[q].x;  // 31/2+0
        c.y = (double)size / 2 + points[q].y;  // 31/2+0

        // 左下
        p[0].x = c.x - Constants::length / 2 / cSize;  // 15.5-13.5
        p[0].y = c.y - Constants::width / 2 / cSize;   // 15.5-7.6
        // 左上
        p[1].x = c.x - Constants::length / 2 / cSize;
        p[1].y = c.y + Constants::width / 2 / cSize;
        // 右上
        p[2].x = c.x + Constants::length / 2 / cSize;
        p[2].y = c.y + Constants::width / 2 / cSize;
        // 右下
        p[3].x = c.x + Constants::length / 2 / cSize;
        p[3].y = c.y - Constants::width / 2 / cSize;

        for (int o = 0; o < Constants::headings; ++o) {  // 72 个 headings

            // 初始化为 cSpace 为 false
            for (int i = 0; i < size; ++i) {  // bbox size: 31x31
                for (int j = 0; j < size; ++j) {
                    cSpace[i * size + j] = false;
                }
            }

            // 绕 z 轴旋转 theta
            for (int j = 0; j < 4; ++j) {
                temp.x = p[j].x - c.x;
                temp.y = p[j].y - c.y;
                nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
                nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
            }

            // 下一个朝向
            theta += Constants::deltaHeadingRad;

            // cell traversal clockwise
            for (int k = 0; k < 4; ++k) {
                // create the vectors clockwise
                if (k < 3) {
                    start = nP[k];
                    end = nP[k + 1];
                }
                else {
                    start = nP[3];
                    end = nP[0];
                }

                // 起始 cell 的 id
                X = (int)start.x;
                Y = (int)start.y;
                cSpace[Y * size + X] = true;
                t.x = end.x - start.x;
                t.y = end.y - start.y;
                stepX = sign(t.x);  // +-1
                stepY = sign(t.y);  // +-1

                // 归一化
                if (fabs(t.x) > 0.000001) {
                    tDeltaX = 1.f / std::abs(t.x);
                    if (stepX > 0) {
                        tMaxX = tDeltaX * (1 - (start.x - (int)start.x));
                    }
                    else {
                        tMaxX = tDeltaX * (start.x - (int)start.x);
                    }
                }
                else {
                    tDeltaX = 1000;
                    tMaxX = 2.0;
                }
                if (fabs(t.y) > 0.000001) {
                    tDeltaY = 1.f / std::abs(t.y);
                    if (stepY > 0) {
                        tMaxY = tDeltaY * (1 - (start.y - (int)start.y));
                    }
                    else {
                        tMaxY = tDeltaY * (start.y - (int)start.y);
                    }
                }
                else {
                    tDeltaY = 1000;
                    tMaxY = 2.0;
                }

                while ((int)end.x != X || (int)end.y != Y) {

                    // only increment x if the t length is smaller and the result will be closer to the goal
                    if (tMaxX < tMaxY && std::abs(X + stepX - (int)end.x) < std::abs(X - (int)end.x)) {
                        tMaxX = tMaxX + tDeltaX;  // 加一格
                        X = X + stepX;
                        cSpace[Y * size + X] = true;
                    }
                    // only increment y if the t length is smaller and the result will be closer to the goal
                    else if (tMaxY < tMaxX && std::abs(Y + stepY - (int)end.y) < std::abs(Y - (int)end.y)) {
                        tMaxY = tMaxY + tDeltaY;
                        Y = Y + stepY;
                        cSpace[Y * size + X] = true;
                    }
                    else if (2 >= std::abs(X - (int)end.x) + std::abs(Y - (int)end.y)) {
                        if (std::abs(X - (int)end.x) > std::abs(Y - (int)end.y)) {
                            X = X + stepX;
                            cSpace[Y * size + X] = true;
                        }
                        else {
                            Y = Y + stepY;
                            cSpace[Y * size + X] = true;
                        }
                    } else {
                        // this SHOULD NOT happen
                        std::cout << "\n--->tie occured, please check for error in script\n";
                        break;
                    }
                }
            }

            // 填充矩形内部
            for (int i = 0; i < size; ++i) {
                inside = false;
                for (int j = 0; j < size; ++j) {
                    for (int k = 0; k < size; ++k) {
                        if (cSpace[i * size + k] && !inside) {
                            hcross1 = k;
                            inside = true;
                        }

                        if (cSpace[i * size + k] && inside) {
                            hcross2 = k;
                        }
                    }
                    if (j > hcross1 && j < hcross2 && inside) {
                        cSpace[i * size + j] = true;
                    }
                }
            }

            // 生成 lookup
            int count = 0;
            for (int i = 0; i < size; ++i) {
                for (int j = 0; j < size; ++j) {
                    if (cSpace[i * size + j]) {
                        // compute the relative position of the car cells
                        lookup[q * Constants::headings + o].pos[count].x = j - (int)c.x;
                        lookup[q * Constants::headings + o].pos[count].y = i - (int)c.y;
                        // add one for the length of the current list
                        count++;
                    }
                }
            }
            lookup[q * Constants::headings + o].length = count;

            if (DEBUG) {
                //DEBUG
                for (int i = 0; i < size; ++i) {
                    std::cout << "\n";
                    for (int j = 0; j < size; ++j) {
                        if (cSpace[i * size + j]) {
                            std::cout << "#";
                        }
                        else {
                            std::cout << ".";
                        }
                    }
                }
                std::cout << "\n\nthe center of " << q* Constants::headings + o << " is at " << c.x << " | " << c.y << std::endl;
                for (int i = 0; i < lookup[q * Constants::headings + o].length; ++i) {
                    std::cout << "[" << i << "]\t" << lookup[q * Constants::headings + o].pos[i].x << " | " << lookup[q * Constants::headings + o].pos[i].y << std::endl;
                }
            }
        }
    }

}

}  // Lookup
}  // HybridAStar
#endif // LOOKUP

