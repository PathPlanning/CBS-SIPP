#ifndef SIPP_H
#define SIPP_H
#include "structs.h"
#include "map.h"
#include "lineofsight.h"
#include "heuristic.h"
#include <unordered_map>
#include <map>
#include <set>
class SIPP
{
public:

    SIPP() { h_values = nullptr;}
    ~SIPP() { }
    Path find_path(Agent agent, const Map &map, std::list<Constraint> cons = {}, Heuristic *h_values = nullptr);

private:
    Agent agent;
    void find_successors(Node curNode, const Map &map, std::list<Node> &succs);
    void add_open(Node newNode);
    Node find_min(int size);
    void generate_moves(int k);
    int  count_h_value(int i, int j, int goal_i, int goal_j);
    double dist(const Node& a, const Node& b);
    void reconstruct_path(Node curNode);
    void make_constraints(std::list<Constraint> &cons);
    void clear();

    unsigned int openSize;
    std::unordered_multimap<int, Node> close;
    std::vector<std::list<Node>> open;
    Path path;
    LineOfSight los;
    Heuristic* h_values;
    std::vector<std::pair<int, int>> moves_2k;
    std::map<Move, std::vector<Move>> constraints;//stores sets of constraints associated with moves
    std::map<std::pair<int, int>, std::vector<std::pair<double, double>>> collision_intervals;//stores sets of collision intervals associated with cells
};

#endif // SIPP_H
