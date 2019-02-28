#include "AStarPlanner.h"

using namespace std;

AStarPlanner::AStarPlanner(VirtualRobot::RobotPtr robot, VirtualRobot::SceneObjectSetPtr obstacles, float cellSize)
 : Planner2D(robot, obstacles), cellSize(cellSize)
{
  // Standart-Konstruktor
}

void AStarPlanner::createUniformGrid()
{
    //+1 for explicit rounding up
    size_t cols = (sceneBoundsMax.x() - sceneBoundsMin.x()) / cellSize + 1;
    size_t rows = (sceneBoundsMax.y() - sceneBoundsMin.y()) / cellSize + 1;

    // build objects in grid
    for (size_t r = 0; r < rows; r++) {
        grid.push_back(std::vector<NodePtr>(cols));
        for (size_t c = 0; c < cols; c++) {
            Eigen::Vector2f pos;
            pos.x() = sceneBoundsMin.x() + c * cellSize + cellSize / 2;
            pos.y() = sceneBoundsMin.y() + r * cellSize + cellSize / 2;
            grid[r][c] = NodePtr(new Node(pos));
        }
    }

    //init valid successors
    for (size_t r = 0; r < rows; r++) {
        for (size_t c = 0; c < cols; c++) {
            NodePtr candidate = grid[r][c];
            if (!fulfillsConstraints(candidate)) continue;
            //add the valid node as successor to all its neighbors
            for (int nR = -1; nR <= 1; nR++) {
                for (int nC = -1; nC <= 1; nC++) {
                    const int neighborIndexX = c + nC;
                    const int neighborIndexY = r + nR;
                    if (neighborIndexX < 0 || neighborIndexY < 0 || (nR == 0 && nC == 0)) continue;

                    grid[neighborIndexY][neighborIndexX]->successors.push_back(candidate);
                }
            }
        }
    }

}

bool AStarPlanner::fulfillsConstraints(NodePtr n)
{
    if (!obstacles) return true;
    robotCollisionModel->setGlobalPose(positionToMatrix4f(n->position));

    // check for collisions
    if (!obstacles) return true;
    robotCollisionModel->setGlobalPose(positionToMatrix4f(n->position));
    Eigen::Vector3f P1, P2;
    int id1, id2;
    for (size_t i = 0; i < obstacles->getSize(); i++) {
        float dist = VirtualRobot::CollisionChecker::getGlobalCollisionChecker()->calculateDistance(robotCollisionModel, obstacles->getSceneObject(i)->getCollisionModel(), P1, P2, &id1, &id2);
        if (dist <= cellSize / 2) return false;
    }
    return n->position.x() >= sceneBoundsMin.x() && n->position.x() <= sceneBoundsMax.x() &&
           n->position.y() >= sceneBoundsMin.y() && n->position.y() <= sceneBoundsMax.y();
}

NodePtr AStarPlanner::closestNode(Eigen::Vector2f v)
{
    size_t r = (v.y() - cellSize / 2 - sceneBoundsMin.y()) / cellSize;
    size_t c = (v.x() - cellSize / 2 - sceneBoundsMin.x()) / cellSize;
    return grid[r][c];
}

float AStarPlanner::heuristic(NodePtr n1, NodePtr n2)
{
    return (n1->position - n2->position).norm();
}

std::vector<Eigen::Vector2f> AStarPlanner::plan(Eigen::Vector2f start, Eigen::Vector2f goal)
{
    grid.clear();
    std::vector<Eigen::Vector2f> result;
    if (!robot || !robot->hasRobotNode(robotColModelName) || !robot->getRobotNode(robotColModelName)->getCollisionModel())
    {
        cout << "No collision model with name " << robotColModelName << " found..." << endl;
        return result;
    }
    robotCollisionModel = robot->getRobotNode(robotColModelName)->getCollisionModel()->clone();
    createUniformGrid();
    NodePtr v_start = closestNode(start);   // Finde den naechsten Knoten zur Startposition des Roboters, da der Graph des Problemgebiets fuer A-Star vorgegeben ist
    NodePtr v_goal = closestNode(goal);     // Analog beim Zielknoten

    // Deklarationen (ohne Initialisierung) -> Datentypen sind fuer die Beispielimplementierung vorgegeben
    // Open set O: nodes to visit
    std::set<NodePtr> openSet;
    // Closed set C: nodes which have been expanded/visited
    std::set<NodePtr> closedSet;
    // Accumulated costs g(v)
    std::map<NodePtr, float> g;             // std::map ist vergleichbar mit Python Dict -> Assoziative Datenstruktur nach Key, Value
                                            // Key = NodePtr, Value = float
    // Heuristic h(v)
    std::map<NodePtr, float> h;
    // Expected cost f(v) = g(v) + h(v)

    ///////////////////////////////////////////////
    // TODO: INSERT CODE HERE

    // Pseudo Code A-Star:
    /*
    O = {v_start}
    C = {}

    g(v_i) = INF, 1 <= i <= x
    g(v_start) = 0

    while (O != {}):
      {
      Finde Knoten v_i zum expandieren:
      argmin v_i in O (f(v_i) = g(v_i) + h(v_i))

      if v_i == v_goal
        {
          done
        }
      o.remove(v_i)
      C.add(v_i)

      Update alle Nachfolger v_j von v_i
        {
          if v_j in C:
            naechstes v_j
          else:
            o.add(v_j)

          if Weg zu v_j ist ueber v_i kleiner als bisher ( g(v_i) + cost(v_i, v_j) < g(v_j) ):
            Update g(v_j) = g(v_i) + cost(v_i, v_j)
            h(v_j) =  heuristic(v_j, v_goal)

            pred(v_j) = v_i  setze neuen kuerzesten vorgaenger von v_j ueber v_i

        }

      }
      */
      
      openSet.insert(v_start)
      // closedSet ist ohnehin leer initialisiert, da ich nichts zur Initialisierung verwendet habe und nur der Konstruktor aufgerufen wurde
      // g wird leer gelassen und bei Verwendung der Werte beschrieben. Das ist effizienter

      g[v_start] = 0;                           // Kosten zum Startknoten sind 0
      h[v_start] = heuristic(v_start, v_goal);  // Berechne Kosten bis zum Ziel des Startknotens. Damit ist der Wert einmal geschrieben und muss spaeter nicht innerhalb einer Schleife bestimmt werden

      while ( !openSet.empty() )
      {
        // Finde den naechsten Knoten zum expandieren.
        // Dazu muessen wir das minimale f[v_i] finden
        // Das koennte man effizienter umsetzen, wenn man mit Pointern/Datenstrukturen auf dem Heap arbeitet
        NodePtr v_i;
        float min_f = FLT_MAX;
        for ( NodePtr v : openSet )
        {
          float f = g[v] + h[v];
          if ( f < min_f )
          {
            min_f = f;
            v_i = v;
          }
        }

        if ( v_i == v_goal )
        {
          break; // Brich (innere) Schleife ab
        }
        openSet.erease(v_i);
        closedSet.insert(v_i);

        for ( NodePtr v_j : v_i->successors  )
        {
          // NodePtr ist ein Iterator-Objekt
          // Der Klasse von v_i hat alle Nachfolgeknoten in der Membervariable succuessors.
          // ->  ist der C++ Zugriff auf Klassenmember mit einem Pointer-Objekt
          if ( closedSet.count(v_j) ) // count zaehlt, wie viele Eintraege es in der Map gibt
          {
            // Wenn schon ein Eintrag fuer v_j existiert, wird die Berechnung uebersprungen
            continue; // naechste Schleifeniteration
          }
          openSet.insert(v_j);

          // Ueberpruefe, ob es einen kuerzeren Pfad zu v_j gibt:
          float g_new = g[v_i] + heuristic(v_i, v_j);       // Heuristik: Euklidische Distanz
          float g_old = g.count(v_j) ? g[v_j] : FLT_MAX;    // ? bedeutet: Nimm den ersten Eintrag hinter dem ?, wenn vor dem ? True entsteht, ansonsten den Eintag hinter dem ?
          if ( g_new < g_old )
          {
            g[v_j] = g_new;
            h[v_j] = heuristic(v_j, v_goal);                // Um nicht immer neu berechnen zu muessen
            v_j->predecessor = v_i;
          }
        }
      }

    ///////////////////////////////////////////////

    //found solution, now retrieve path from goal to start
    if (v_goal)
        result = v_goal->traversePredecessors();
    //since the graph was traversed from goal to start, we have to reverse the order
    std::reverse(result.begin(), result.end());

    return result;
}
