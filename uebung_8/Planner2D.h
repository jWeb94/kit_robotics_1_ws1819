#ifndef PLANNER2D_H
#define PLANNER2D_H

#include <boost/shared_ptr.hpp>
#include <VirtualRobot/SceneObjectSet.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/Robot.h>
#include <Eigen/Geometry>

class Planner2D
{
public:
    /*!
        Initialize Planner with robot and obstacles.
        \param robot The robot
        \param obstacles The obstacles which should be considered by the planner
    */
    Planner2D(VirtualRobot::RobotPtr robot, VirtualRobot::SceneObjectSetPtr obstacles = VirtualRobot::SceneObjectSetPtr());


    // planners implement this method
    virtual std::vector<Eigen::Vector2f> plan(Eigen::Vector2f start, Eigen::Vector2f goal) = 0;
    
    //! Update obstacles
    void setObstacles(VirtualRobot::SceneObjectSetPtr obstacles);

    //! update robot
    void setRobot(VirtualRobot::RobotPtr robot);

    //! set name of RobotNode which should be used for collision detection
    void setRobotColModel(const std::string &robotColModelName);

    Eigen::Matrix4f positionToMatrix4f(Eigen::Vector2f pos);

    //! set a float parameter that is identified with a string
    void setParameter(const std::string &s, float p);
    
    //! check if a parameter is set
    bool hasParameter(const std::string &s);
    
    //! get the corresponding float parameter (0 is returned when parameter string is not present)
    float  getParameter(const std::string &s);

protected:
    std::string robotColModelName;
    //local copy of the robot's collision model that can be moved around without moving the robot
    VirtualRobot::CollisionModelPtr robotCollisionModel;
    VirtualRobot::RobotPtr robot;
    VirtualRobot::SceneObjectSetPtr obstacles;
    Eigen::Vector2f sceneBoundsMin;
    Eigen::Vector2f sceneBoundsMax;

    std::map<std::string, float> parameters;
};

typedef boost::shared_ptr<Planner2D> Planner2DPtr;

#endif // PLANNER2D_H
