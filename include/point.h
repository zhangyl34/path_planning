#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

/* 为 bucketedequeue.h 提供服务*/

#define INTPOINT IntPoint

namespace HybridAStar {
/* A light-weight integer point with fields x,y */
class IntPoint {
public:
    IntPoint() : x(0), y(0) {}
    IntPoint(int _x, int _y) : x(_x), y(_y) {}
    int x, y;
};
}
#endif
