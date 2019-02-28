#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include "Planner2D.h"
#include "Node.h"

/*!
    A RRT planner.
*/
class RRTPlanner : public Planner2D
{
public:
    RRTPlanner( VirtualRobot::RobotPtr robot,
                VirtualRobot::SceneObjectSetPtr obstacles = VirtualRobot::SceneObjectSetPtr(),
                float goalReachedEpsilon = 200.0f,
                float incrementalDistance = 50.f,
                float randomNodeGoal = 0.01f);
    std::vector<Eigen::Vector2f> plan(Eigen::Vector2f start, Eigen::Vector2f goal);

private:
    //! how close the robot should be to the goal
    float goalReachedEpsilon;
    //! how far a node is extended in each iteration
    float incrementalDistance;
    //! a small chance to select the goal when creating a random node
    float randomNodeGoal;
    std::vector<NodePtr> vertices;

    //! create a random real number in the range [0,1)
    float randomFloat();

    NodePtr createRandomNode(NodePtr nodeGoal, float randomNodeGoal);
    NodePtr nearestVertex(NodePtr nodeRandom);
    NodePtr extend(NodePtr nodeNear, NodePtr nodeRandom, float incrementalDistance);
    bool fulfillsConstraints(NodePtr nodeNew);
    void addVertex(NodePtr nodeNew);
    void addEdge(NodePtr nodeNear, NodePtr nodeNew);
};

#endif // RRTPLANNER_H
