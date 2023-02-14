#ifndef CONSTANTS
#define CONSTANTS

// HEADING => 0 - 359 degrees, 0 being north pointing towards positive Y
// X-COORDINATE => designating the width of the grid
// Y-COORDINATE => designating the height of the grid

#include <cmath>

namespace HybridAStar {
namespace Constants {

// The cell size of the 2D grid of the world
static const float cellSize = 0.05;  // 即 mapResolution，单位：m
static const float cellNum = 400;    // 即地图格数，单位：格
static const float pcRadius = 0.2;   // 点云拓展为球体，也可以理解为机器人半径，用于防碰撞，单位：m

// 使用 reedsShepp 曲线
static const bool reverse = true;
// 使用 dubins 曲线
static const bool dubins = false;
// 使用 dubins 曲线去命中目标
static const bool dubinsShot = true;
// 启发式是否包含 a* 2D 搜索
static const bool twoD = false;

static const int iterations = 30000;    // 最大循环次数
static const double width = 0.76;       // 小车尺寸，单位：m
static const double length = 1.35;      // 小车尺寸，单位：m
static const float r = 2 / cellSize;  // 最小弯转半径，单位：格
static const int headings = 72;         // 72 个朝向
static const float deltaHeadingDeg = 360 / (float)headings;
static const float deltaHeadingRad = 2 * M_PI / (float)headings;
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;


/**
 * The tie breaker breaks ties between nodes expanded in the same cell
 * As the cost-so-far are bigger than the cost-to-come it is reasonbale to believe that the algorithm would prefer the predecessor rather than the successor.
 * This would lead to the fact that the successor would never be placed and the the one cell could only expand one node. The tieBreaker artificially increases the cost of the predecessor
 * to allow the successor being placed in the same cell.
**/
static const float tieBreaker = 0.01;

/* 启发式*/
/// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
static const float factor2D = sqrt(5) / sqrt(2) + 1;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
static const float penaltyTurning = 1.05;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
static const float penaltyReversing = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
static const float penaltyCOD = 2.0;
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
static const float dubinsShotDistance = 100;  // 触发 dubinsShot 的距离阈值，单位：格
static const float dubinsStepSize = 10;  // dubins 碰撞检测步长，单位：格

/* collision lookup*/
// The bounding box size length and width to precompute all possible headings
static const int bbSize = std::ceil((sqrt(width * width + length* length)) / cellSize);  // ceil(1.55/0.05)=31
// The sqrt of the number of discrete positions per cell
static const int positionResolution = 1;
// The number of discrete positions per cell
static const int positions = positionResolution * positionResolution;
// A structure describing the relative position of the occupied cell based on the center of the vehicle
struct relPos {
    // the x position relative to the center
    int x;
    // the y position relative to the center
    int y;
};
// A structure capturing the lookup for each theta configuration
struct config {
    int length;        // 实际使用的大小
    relPos pos[1600];  // 预留的大小为 bbSize*bbSize
};


// SMOOTHER SPECIFIC
/// [m] --- The minimum width of a safe road for the vehicle at hand
static const float minRoadWidth = 2;


// COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
/// A structure to express colors in RGB values
struct color {
    /// the red portion of the color
    float red;
    /// the green portion of the color
    float green;
    /// the blue portion of the color
    float blue;
};
/// A definition for a color used for visualization
static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
/// A definition for a color used for visualization
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
/// A definition for a color used for visualization
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
/// A definition for a color used for visualization
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
/// A definition for a color used for visualization
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
}
}

#endif // CONSTANTS

