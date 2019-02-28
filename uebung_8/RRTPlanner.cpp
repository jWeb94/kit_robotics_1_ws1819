#include "RRTPlanner.h"
#include <cstdlib>
using namespace std;

RRTPlanner::RRTPlanner(VirtualRobot::RobotPtr robot, VirtualRobot::SceneObjectSetPtr obstacles, float goalReachedEpsilon, float incrementalDistance, float randomNodeGoal)
    : Planner2D(robot, obstacles), goalReachedEpsilon(goalReachedEpsilon), incrementalDistance(incrementalDistance), randomNodeGoal(randomNodeGoal)
{
    srand(time(0));
}

float RRTPlanner::randomFloat()
{
    return (float)rand() / RAND_MAX;
}

NodePtr RRTPlanner::createRandomNode(NodePtr nodeGoal, float randomNodeGoal)
{
    if (randomFloat() < randomNodeGoal) {
        return nodeGoal;
    }
    Eigen::Vector2f resultPos;
    resultPos.x() = sceneBoundsMin.x() + (sceneBoundsMax.x() - sceneBoundsMin.x()) * randomFloat();
    resultPos.y() = sceneBoundsMin.y() + (sceneBoundsMax.y() - sceneBoundsMin.y()) * randomFloat();
    return NodePtr(new Node(resultPos));
}

NodePtr RRTPlanner::nearestVertex(NodePtr nodeRandom)
{
    if (vertices.empty()) return NodePtr();

    NodePtr nearest = vertices[0];

    ///////////////////////////////////////////////
    // TODO: INSERT CODE HERE

    // Finde den naechsten Nachbarn des aktuell betrachteten Knoten

    for ( NodePtr node : vertices )
    {
      Eigen::Vector2f distanceVector = nodeRandom->position - node->position;
      float distance = distanceVector.norm();
      if(distance < minDistance)
      {
        nearest = node;
        minDistance = distance;
      }
    }






    ///////////////////////////////////////////////

    return nearest;
}

NodePtr RRTPlanner::extend(NodePtr nodeNear, NodePtr nodeRandom, float incrementalDistance)
{
    Eigen::Vector2f newPos = Eigen::Vector2f::Zero();

    ///////////////////////////////////////////////
    // INSERT CODE HERE

    // Gehe auf Geraden in Richtung von Naechstem Nachbarn zum neuen, zufaelligen Knoten mit der Schrittweite incrementalDistance

    EigenVector2f fromNearToRandom = nodeRandom->positin - nodeNear->position;
    float actualDistance = fromNearToRandom.norm();

    Eigen::Vector2f increment = fromNearToRandom.normalized() * std::min(actualDistance, incrementalDistance);

    newPos = nodeNear->position + increment;

    ///////////////////////////////////////////////

    return NodePtr(new Node(newPos));
}

bool RRTPlanner::fulfillsConstraints(NodePtr nodeNew)
{
    if (!obstacles) return true;
    robotCollisionModel->setGlobalPose(positionToMatrix4f(nodeNew->position));

    ///////////////////////////////////////////////
    // TODO: INSERT CODE HERE

    // Kollisionspruefung
    // Wenn Kollision erkannt: false, ansonsten true als Rueckgabewert
    VirtualRobot::CollisionCheckerPtr collisionChecker = VirtualRobot::CollisionChecker::getGlobalCollisionChecker();

    for ( VirtualRobot::CollisionModelPtr obstacle : obstacle->getCollisionModels() )
    {
      bool collision = collisionChecker->checkCollision(robotCollisionModel, obstacle);

      if ( collision )
      {
        return false;
      }
    }

    ///////////////////////////////////////////////

    return true;
}

void RRTPlanner::addVertex(NodePtr nodeNew)
{
    vertices.push_back(nodeNew);
}

void RRTPlanner::addEdge(NodePtr nodeNear, NodePtr nodeNew)
{
    nodeNew->predecessor = nodeNear;
    nodeNear->successors.push_back(nodeNew);
}

vector<Eigen::Vector2f> RRTPlanner::plan(Eigen::Vector2f start, Eigen::Vector2f goal)
{
    vertices.clear();
    vector<Eigen::Vector2f> result;
    addVertex(NodePtr(new Node(start)));
    if (!robot || !robot->hasRobotNode(robotColModelName) || !robot->getRobotNode(robotColModelName)->getCollisionModel())
    {
        cout << "No collision model with name " << robotColModelName << " found..." << endl;
        return result;
    }
    robotCollisionModel = robot->getRobotNode(robotColModelName)->getCollisionModel()->clone();
    bool foundSolution = false;
    int step = 0;
    NodePtr nodeGoal = NodePtr(new Node(goal));
    while (!foundSolution)
    {
        step++;
        if (step > 10000)
        {
            std::cout << "step > 10000, aborting..." << std::endl;
            return result;
        }

        ///////////////////////////////////////////////
        // TODO: INSERT CODE HERE

        NodePtr nodeRandom = createRandomNode(nodeGoal, randomNodeGoal); // Zufaelligen Knoten sampeln
        NodePtr nodeNear = nearestVertex(nodeRandom);
        NodePtr nodeNew = extend(nodeNear, nodeRandom, incrementalDistance);

        if ( fulfillsConstraints(nodeNew) )
        {
          addVertex(nodeNew);
          addEdge(nodeNear, nodeNew);

          float distanceToGoal = (nodeNew->position - goal).norm();

          if ( distanceToGoal < goalReachedEpsilon )
          {
            foundSolution = true;
            nodeGoal = nodeNew; 
          }
        }









        ///////////////////////////////////////////////
    }

    //found solution, now retrieve path from goal to start
    if (nodeGoal)
        result = nodeGoal->traversePredecessors();
    //since the graph was traversed from goal to start, we have to reverse the order
    std::reverse(result.begin(), result.end());
    return result;
}
