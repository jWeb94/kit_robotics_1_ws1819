#ifndef ASTARPLANNER_H
#define ASTARPLANNER_H

#include "Planner2D.h"
#include "Node.h"

/*!
    The A* planner
*/
class AStarPlanner : public Planner2D
{
public:
    AStarPlanner( VirtualRobot::RobotPtr robot,
                  VirtualRobot::SceneObjectSetPtr obstacles = VirtualRobot::SceneObjectSetPtr(),
                  float cellSize = 100.f);
    std::vector<Eigen::Vector2f> plan(Eigen::Vector2f start, Eigen::Vector2f goal);

private:
    //! how big each cell is in the uniform grid
    float cellSize;
    std::vector<std::vector<NodePtr> > grid;
    void createUniformGrid();
    bool fulfillsConstraints(NodePtr n);
    NodePtr closestNode(Eigen::Vector2f v);
    float heuristic(NodePtr n1, NodePtr n2);
};

#endif // ASTARPLANNER_H
