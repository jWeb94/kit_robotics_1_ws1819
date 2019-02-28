#include "Planner2D.h"
#include <algorithm>
#include <cmath>

Planner2D::Planner2D(VirtualRobot::RobotPtr robot, VirtualRobot::SceneObjectSetPtr obstacles)
{
    sceneBoundsMin.setZero();
    sceneBoundsMax.setZero();
    setRobot(robot);
    setObstacles(obstacles);
}

void Planner2D::setObstacles(VirtualRobot::SceneObjectSetPtr obstacles) {
    this->obstacles = obstacles;
    if (obstacles) {
        for (size_t i = 0; i < obstacles->getCollisionModels().size(); i++) {
            VirtualRobot::BoundingBox bb = obstacles->getCollisionModels()[i]->getBoundingBox();
            sceneBoundsMin.x() = std::min(bb.getMin().x(), sceneBoundsMin.x());
            sceneBoundsMin.y() = std::min(bb.getMin().y(), sceneBoundsMin.y());
            sceneBoundsMax.x() = std::max(bb.getMax().x(), sceneBoundsMax.x());
            sceneBoundsMax.y() = std::max(bb.getMax().y(), sceneBoundsMax.y());
        }
    }
}

void Planner2D::setRobot(VirtualRobot::RobotPtr robot) {
    this->robot = robot;
}

void Planner2D::setRobotColModel(const std::string &robotColModelName) {
    this->robotColModelName = robotColModelName;
}

Eigen::Matrix4f Planner2D::positionToMatrix4f(Eigen::Vector2f pos) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    result(0, 3) = pos.x();
    result(1, 3) = pos.y();
    return result;
}

void Planner2D::setParameter(const std::string &s, float p)
{
    parameters[s] = p;
}

bool Planner2D::hasParameter(const std::string &s)
{
    return (parameters.find(s) != parameters.end());
}

float Planner2D::getParameter(const std::string &s)
{
    if (!hasParameter(s))
    {
        std::cout << "Warning, parameter " << s << " not set, returning 0" << std::endl;
        return 0.0f;
    }
    return parameters[s];
}
