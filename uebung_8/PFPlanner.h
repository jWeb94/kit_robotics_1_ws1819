#ifndef PFPLANNER_H
#define PFPLANNER_H

#include "Planner2D.h"
/*!
    The potential field planner
 */
class PFPlanner : public Planner2D
{
public:
    PFPlanner(  VirtualRobot::RobotPtr robot, 
                VirtualRobot::SceneObjectSetPtr obstacles = VirtualRobot::SceneObjectSetPtr(), 
                float goalReachedEpsilon = 100.0f, 
                float cAtt = 1.0f, 
                float cRep = 10.0f,
                float cAreaOfInfluence = 500.0f);
    std::vector<Eigen::Vector2f> plan(Eigen::Vector2f start, Eigen::Vector2f goal);

private:
    float goalReachedEpsilon;
    //! constant for the attracting force
    float cAtt;
    //! constant for the repulsive force
    float cRep;
    //! how far an obstacle emits a repulsive
    float cAreaOfInfluence;

    Eigen::Vector2f calcAttractingForce(Eigen::Vector2f currentPosition, Eigen::Vector2f goal);
    Eigen::Vector2f calcRepulsiveForce(Eigen::Vector2f currentPosition, VirtualRobot::SceneObjectPtr obstacle);
};

#endif // PFPLANNER_H
