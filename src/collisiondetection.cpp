#include "collisiondetection.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection() {}

/* 3D 碰撞检测，如果无碰撞，返回 true。*/
bool CollisionDetection::configurationTest(float x, float y, float t) const {
    
    // int X = (int)x;
    // int Y = (int)y;
    // // 当前 cell 内 X 方向的 id
    // int iX = (int)((x - (int)x) * Constants::positionResolution);  // 分辨率为 1
    // iX = iX > 0 ? iX : 0;
    // // 当前 cell 内 Y 方向的 id
    // int iY = (int)((y - (int)y) * Constants::positionResolution);  // 分辨率为 1
    // iY = iY > 0 ? iY : 0;
    // // 72 个朝向的 id
    // int iT = (int)(t / Constants::deltaHeadingRad);
    // // 总 id
    // int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
    
    // int cX;
    // int cY;
    // // collisionLookup 根据小车的尺寸扩充一些节点（矩形框），检查这些节点是否与环境碰撞
    // for (int i = 0; i < collisionLookup[idx].length; ++i) {
    //     cX = (X + collisionLookup[idx].pos[i].x);
    //     cY = (Y + collisionLookup[idx].pos[i].y);

    //     // make sure the configuration coordinates are actually on the grid
    //     if (cX >= 0 && (unsigned int)cX < grid.info.width && cY >= 0 && (unsigned int)cY < grid.info.height) {
    //         if (grid.data[cY * grid.info.width + cX]==100) {
    //             return false;
    //         }
    //     }
    // }

    // three circle collision check
    int cX = static_cast<int>(x);  // cell
    int cY = static_cast<int>(y);
    if (cX >= 0 && cX < grid.info.width && cY >= 0 && cY < grid.info.height) {
        if (grid.data[cY * grid.info.width + cX] == 100) {
            return false;
        }
    }
    else {
        std::cout << "cart is out of the map? s1"  << std::endl;
    }
    int dCircle = ceil((Constants::length-Constants::width)/2.0);  // cell
    cX = static_cast<int>(x+dCircle*cos(t));  // t = rad
    cY = static_cast<int>(y+dCircle*sin(t));
    if (cX >= 0 && cX < grid.info.width && cY >= 0 && cY < grid.info.height) {
        if (grid.data[cY * grid.info.width + cX] == 100) {
            return false;
        }
    }
    else {
        std::cout << "cart is out of the map? 2"  << std::endl;
    }
    cX = static_cast<int>(x-dCircle*cos(t));  // t = rad
    cY = static_cast<int>(y-dCircle*sin(t));
    if (cX >= 0 && cX < grid.info.width && cY >= 0 && cY < grid.info.height) {
        if (grid.data[cY * grid.info.width + cX] == 100) {
            return false;
        }
    }
    else {
        std::cout << "cart is out of the map? 3"  << std::endl;
    }

    return true;
}