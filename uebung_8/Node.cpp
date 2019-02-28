#include "Node.h"

Node::Node(Eigen::Vector2f position) : position(position)
{
}

std::vector<Eigen::Vector2f> Node::traversePredecessors()
{
    std::vector<Eigen::Vector2f> result;
    result.push_back(position);
    NodePtr predecessor = this->predecessor;
    while (predecessor) {
        result.push_back(predecessor->position);
        predecessor = predecessor->predecessor;
    }
    return result;
}
