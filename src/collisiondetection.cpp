#include "collisiondetection.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
    Lookup::collisionLookup(collisionLookup);
}

/* 3D 碰撞检测*/
bool CollisionDetection::configurationTest(float x, float y, float t) const {
    
    int X = (int)x;
    int Y = (int)y;
    // 当前 cell 内 X 方向的 id
    int iX = (int)((x - (int)x) * Constants::positionResolution);  // 分辨率为 1
    iX = iX > 0 ? iX : 0;
    // 当前 cell 内 Y 方向的 id
    int iY = (int)((y - (int)y) * Constants::positionResolution);  // 分辨率为 1
    iY = iY > 0 ? iY : 0;
    // 72 个朝向的 id
    int iT = (int)(t / Constants::deltaHeadingRad);
    // 总 id
    int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
    
    int cX;
    int cY;
    // collisionLookup 根据小车的尺寸扩充一些节点（矩形框），检查这些节点是否与环境碰撞
    for (int i = 0; i < collisionLookup[idx].length; ++i) {
        cX = (X + collisionLookup[idx].pos[i].x);
        cY = (Y + collisionLookup[idx].pos[i].y);

        // make sure the configuration coordinates are actually on the grid
        if (cX >= 0 && (unsigned int)cX < grid.info.width && cY >= 0 && (unsigned int)cY < grid.info.height) {
            if (grid.data[cY * grid.info.width + cX]==100) {
                return false;
            }
        }
    }

    return true;
}