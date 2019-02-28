#include "PFPlanner.h"
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
using namespace std;

PFPlanner::PFPlanner(VirtualRobot::RobotPtr robot, VirtualRobot::SceneObjectSetPtr obstacles, float goalReachedEpsilon, float cAtt, float cRep, float cAreaOfInfluence)
: Planner2D(robot, obstacles), goalReachedEpsilon(goalReachedEpsilon), cAtt(cAtt), cRep(cRep), cAreaOfInfluence(cAreaOfInfluence)
{

}

Eigen::Vector2f PFPlanner::calcAttractingForce(Eigen::Vector2f currentPosition, Eigen::Vector2f goal) 
{
    Eigen::Vector2f result(0.f, 0.f);

    ////////////////////////////////////////////////////
    // enter code here
    ////////////////////////////////////////////////////
    return result;
}

Eigen::Vector2f PFPlanner::calcRepulsiveForce(Eigen::Vector2f currentPosition, VirtualRobot::SceneObjectPtr obstacle)
{
    Eigen::Vector2f result(0.f, 0.f);
    if (!robotCollisionModel || !obstacle)
        return result;
    robotCollisionModel->setGlobalPose(positionToMatrix4f(currentPosition));

    ////////////////////////////////////////////////////
    // enter code here
    ////////////////////////////////////////////////////

    return result;
}

std::vector<Eigen::Vector2f> PFPlanner::plan(Eigen::Vector2f start, Eigen::Vector2f goal)
{
    float maxStep = 100.0f;
    int loop=0;
    std::vector<Eigen::Vector2f> result;
    if (!robot || !robot->hasRobotNode(robotColModelName) || !robot->getRobotNode(robotColModelName)->getCollisionModel())
    {
        cout << "No collision model with name " << robotColModelName << " found..." << endl;
        return result;
    }
    robotCollisionModel = robot->getRobotNode(robotColModelName)->getCollisionModel()->clone();
    Eigen::Vector2f iterationPosition = start;

    result.push_back(start);

    while ((iterationPosition - goal).norm() > goalReachedEpsilon)
    {
        ////////////////////////////////////////////////////
        // enter code here
        ////////////////////////////////////////////////////

        if (loop>100)
        {
            cout << "loop>100, aborting..." << endl;
            return result;
        }
    }

    return result;
}
