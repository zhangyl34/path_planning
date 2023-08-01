#include "path.h"

using namespace HybridAStar;

void Path::clear() {
    Node3D node;
    path.poses.clear();
    pathNodes.markers.clear();
    pathVehicles.markers.clear();
    addNode(node, 0);
    addVehicle(node, 1);
    publishPath();
    publishPathNodes();
    publishPathVehicles();
}

void Path::updatePath(const std::vector<Node3D>& nodePath) {
    path.header.stamp = ros::Time::now();
    int k = 0;

    for (size_t i = 0; i < nodePath.size(); ++i) {
        addSegment(nodePath[i]);
        addNode(nodePath[i], k);
        k++;
        addVehicle(nodePath[i], k);
        k++;
    }

    return;
}

void Path::addSegment(const Node3D& node) {
    geometry_msgs::PoseStamped vertex;
    vertex.pose.position.x = node.getX() * Constants::cellSize;
    vertex.pose.position.y = node.getY() * Constants::cellSize;
    vertex.pose.position.z = 0;
    vertex.pose.orientation.x = 0;
    vertex.pose.orientation.y = 0;
    vertex.pose.orientation.z = 0;
    vertex.pose.orientation.w = 0;
    path.poses.push_back(vertex);
}

void Path::addNode(const Node3D& node, int i) {
    visualization_msgs::Marker pathNode;

    // delete all previous markers
    if (i == 0) {
        pathNode.action = 3;
    }

    pathNode.header.frame_id = "map";
    pathNode.header.stamp = ros::Time(0);
    pathNode.id = i;
    pathNode.type = visualization_msgs::Marker::SPHERE;
    pathNode.scale.x = 0.1;
    pathNode.scale.y = 0.1;
    pathNode.scale.z = 0.1;
    pathNode.color.a = 1.0;

    if (smoothed) {
        pathNode.color.r = Constants::pink.red;
        pathNode.color.g = Constants::pink.green;
        pathNode.color.b = Constants::pink.blue;
    } else {
        pathNode.color.r = Constants::purple.red;
        pathNode.color.g = Constants::purple.green;
        pathNode.color.b = Constants::purple.blue;
    }

    pathNode.pose.position.x = node.getX() * Constants::cellSize;
    pathNode.pose.position.y = node.getY() * Constants::cellSize;
    pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D& node, int i) {
    visualization_msgs::Marker pathVehicle;

    // delete all previous markersg
    if (i == 1) {
        pathVehicle.action = 3;
    }

    pathVehicle.header.frame_id = "map";
    pathVehicle.header.stamp = ros::Time(0);
    pathVehicle.id = i;
    pathVehicle.type = visualization_msgs::Marker::CUBE;
    pathVehicle.scale.x = Constants::length;
    pathVehicle.scale.y = Constants::width;
    pathVehicle.scale.z = 1;
    pathVehicle.color.a = 0.1;

    if (smoothed) {
        pathVehicle.color.r = Constants::orange.red;
        pathVehicle.color.g = Constants::orange.green;
        pathVehicle.color.b = Constants::orange.blue;
    } else {
        pathVehicle.color.r = Constants::teal.red;
        pathVehicle.color.g = Constants::teal.green;
        pathVehicle.color.b = Constants::teal.blue;
    }

    pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
    pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
    pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    pathVehicles.markers.push_back(pathVehicle);
}
