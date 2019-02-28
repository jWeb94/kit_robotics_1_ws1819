#ifndef NODE_H
#define NODE_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Geometry>
#include <vector>

class Node;
typedef boost::shared_ptr<Node> NodePtr;

/*!
    A Node can store data to all valid neighbors (successors) and a precessor.
    It oers methods to determine the complete path to the starting point.
*/
class Node
{
public:
    Node(Eigen::Vector2f position);

    Eigen::Vector2f position;
    //! all nodes that are adjacent to this one
    std::vector<NodePtr> successors;
    //! for traversal
    NodePtr predecessor;

    //! Collects all predecessors in order to generate path to starting point
    std::vector<Eigen::Vector2f> traversePredecessors();
};

#endif // NODE_H
