#include "smoother.h"
using namespace HybridAStar;

// 检测是否由前进变成倒车
inline bool isCusp(const std::vector<Node3D>& path, int i) {
    bool revim2 = path[i - 2].getPrim() > 3;
    bool revim1 = path[i - 1].getPrim() > 3;
    bool revi   = path[i].getPrim() > 3;
    bool revip1 = path[i + 1].getPrim() > 3;

    return (revim2 != revim1 || revim1 != revi || revi != revip1);
}

void Smoother::smoothPath(DynamicVoronoi& voronoi) {

    // 加载维诺图
    this->voronoi = voronoi;
    this->width = voronoi.getSizeX();
    this->height = voronoi.getSizeY();
    // 迭代次数
    int iterations = 0;
    int maxIterations = 500;
    // 节点个数
    int pathLength = 0;
    pathLength = path.size();
    // 新路径
    std::vector<Node3D> newPath = path;
    // 总权重
    float totalWeight = wSmoothness + wCurvature + wObstacle;

    // 迭代优化
    while (iterations < maxIterations) {
        // 遍历所有节点
        for (int i = 2; i < pathLength - 2; ++i) {

            Vector2D xim2(newPath[i - 2].getX(), newPath[i - 2].getY());
            Vector2D xim1(newPath[i - 1].getX(), newPath[i - 1].getY());
            Vector2D xi(newPath[i].getX(), newPath[i].getY());
            Vector2D xip1(newPath[i + 1].getX(), newPath[i + 1].getY());
            Vector2D xip2(newPath[i + 2].getX(), newPath[i + 2].getY());
            Vector2D correction(0, 0);

            // 若由前进变成倒车，则不优化路径
            if (isCusp(newPath, i)) { continue; }

            correction = correction - obstacleTerm(xi);
            if (!isOnGrid(xi + correction)) { continue; }
            correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
            if (!isOnGrid(xi + correction)) { continue; }
            correction = correction - curvatureTerm(xim2, xim1, xi, xip1, xip2);
            if (!isOnGrid(xi + correction)) { continue; }
            // std::cout << "descent gradient: " << correction/totalWeight << std::endl;

            // 梯度下降
            xi = xi + alpha * correction/totalWeight;  // 单位：格
            newPath[i].setX(xi.getX());
            newPath[i].setY(xi.getY());
            Vector2D Dxi = xi - xim1;
            newPath[i - 1].setT(std::atan2(Dxi.getY(), Dxi.getX()));
        }
        iterations++;
    }
    path = newPath;
}

void Smoother::tracePath(const Node3D* node, int i, std::vector<Node3D> path) {
    
    if (node == nullptr) {
        this->path = path;
        return;
    }

    i++;
    path.push_back(*node);
    tracePath(node->getPred(), i, path);
}

Vector2D Smoother::obstacleTerm(Vector2D xi) {
    Vector2D gradient(0, 0);
    // 从 xi 到最近障碍点的距离
    float obsDst = voronoi.getDistance(xi.getX(), xi.getY());

    int x = (int)xi.getX();
    int y = (int)xi.getY();
    // if the node is within the map
    if (x < width && x >= 0 && y < height && y >= 0) {
        // 从 xi 到最近障碍点的向量
        Vector2D obsVct(xi.getX() - voronoi.data[x][y].obstX,
                        xi.getY() - voronoi.data[x][y].obstY);

        if (obsDst < obsDMax) {
            return gradient = wObstacle * 2 * (obsDst - obsDMax) * obsVct / obsDst;
        }
    }
    return gradient;
}

Vector2D Smoother::curvatureTerm(Vector2D x_im2, Vector2D x_im1, Vector2D x_i, Vector2D x_ip1, Vector2D x_ip2) {
    
    Vector2D gradient;
    // the vectors between the nodes
    const Vector2D& delta_x_im1 = x_im1 - x_im2;
    const Vector2D& delta_x_i = x_i - x_im1;
    const Vector2D& delta_x_ip1 = x_ip1 - x_i;
    const Vector2D& delta_x_ip2 = x_ip2 - x_ip1;

    // ensure that the absolute values are not null
    if (delta_x_im1.length() > 0 && delta_x_i.length() > 0 && delta_x_ip1.length() > 0 && delta_x_ip2.length() > 0) {
        // 公式 (2)
        auto compute_kappa = [](const Vector2D& delta_x_0, const Vector2D& delta_x_1, float& delta_phi, float& kappa) {
            delta_phi = std::acos(Helper::clamp(delta_x_0.dot(delta_x_1) / (delta_x_0.length() * delta_x_1.length()), -1, 1));
            kappa = delta_phi / delta_x_0.length();
        };
        float delta_phi_im1, kappa_im1;
        compute_kappa(delta_x_im1, delta_x_i, delta_phi_im1, kappa_im1);
        float delta_phi_i, kappa_i;
        compute_kappa(delta_x_i, delta_x_ip1, delta_phi_i, kappa_i);
        float delta_phi_ip1, kappa_ip1;
        compute_kappa(delta_x_ip1, delta_x_ip2, delta_phi_ip1, kappa_ip1);

        // if the curvature is smaller then the maximum do nothing
        if (kappa_i <= kappaMax) {
            Vector2D zeros(0, 0);
            return zeros;
        }
        
        // frac{partial{delta_phi}, partial{cos{delta_phi}}}
        auto compute_d_delta_phi = [](const float delta_phi) {
            return -1. / std::sqrt(1. - std::pow(std::cos(delta_phi), 2));
        };

        const float& d_delta_phi_im1 = compute_d_delta_phi(delta_phi_im1);
        const Vector2D& d_cos_delta_phi_im1 = delta_x_im1.ort(delta_x_i) / (delta_x_im1.length() * delta_x_i.length());
        const Vector2D& d_kappa_im1 = 1. / delta_x_im1.length() * d_delta_phi_im1 * d_cos_delta_phi_im1;
        const Vector2D& kim1 = 2. * (kappa_im1 - kappaMax) * d_kappa_im1;

        const float& d_delta_phi_i = compute_d_delta_phi(delta_phi_i);
        // -p1+p2
        const Vector2D& d_cos_delta_phi_i = delta_x_ip1.ort(delta_x_i) / (delta_x_ip1.length() * delta_x_i.length()) -
                                            delta_x_i.ort(delta_x_ip1) / (delta_x_i.length() * delta_x_ip1.length());
        
        const Vector2D& d_kappa_i = 1. / delta_x_i.length() * d_delta_phi_i * d_cos_delta_phi_i -
                                    delta_phi_i / std::pow(delta_x_i.length(), 3) * delta_x_i;
        const Vector2D& ki = 2. * (kappa_i - kappaMax) * d_kappa_i;

        const float& d_delta_phi_ip1 = compute_d_delta_phi(delta_phi_ip1);
        const Vector2D& d_cos_delta_phi_ip1 = -delta_x_ip2.ort(delta_x_ip1) / (delta_x_ip2.length() * delta_x_ip1.length());
        const Vector2D& d_kappa_ip1 = 1. / delta_x_ip1.length() * d_delta_phi_ip1 * d_cos_delta_phi_ip1 +
                                        delta_phi_ip1 / std::pow(delta_x_ip1.length(), 3) * delta_x_ip1;
        const Vector2D& kip1 = 2. * (kappa_ip1 - kappaMax) * d_kappa_ip1;

        // calculate the gradient
        gradient = wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

        if (std::isnan(gradient.getX()) || std::isnan(gradient.getY())) {
            std::cout << "nan values in curvature term" << std::endl;
            Vector2D zeros(0, 0);
            return zeros;
        }

        return gradient;
    }
    else {
        std::cout << "abs values not larger than 0" << std::endl;
        Vector2D zeros(0, 0);
        return zeros;
    }
}

Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
    return wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}

