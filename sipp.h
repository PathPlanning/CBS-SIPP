#ifndef SIPP_H
#define SIPP_H
#include "structs.h"
#include "map.h"
#include "heuristic.h"
#include <unordered_map>
#include <map>
#include <set>
class SIPP
{
public:

    SIPP()  { h_values = nullptr; }
    ~SIPP() { h_values = nullptr; }
    Path find_path(Agent agent, const Map &map, std::list<Constraint> cons = {}, Heuristic *h_values = nullptr);

private:
    Agent agent;
    void find_successors(Node curNode, const Map &map, std::list<Node> &succs);
    void add_open(Node newNode);
    Node find_min(int size);
    double dist(const Node& a, const Node& b);
    void reconstruct_path(Node curNode);
    void make_constraints(std::list<Constraint> &cons);
    void clear();

    unsigned int openSize;
    std::unordered_multimap<int, Node> close;
    std::vector<std::list<Node>> open;
    Path path;
    Heuristic* h_values;
    std::map<Move, std::vector<Move>> constraints;//stores sets of constraints associated with moves
    std::map<std::pair<int, int>, std::vector<std::pair<double, double>>> collision_intervals;//stores sets of collision intervals associated with cells
};

#endif // SIPP_H
