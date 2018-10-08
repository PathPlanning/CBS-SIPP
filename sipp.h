#ifndef SIPP_H
#define SIPP_H
#include "structs.h"
#include "map.h"
#include <unordered_map>
#include <map>
#include <set>
class SIPP
{
public:

    SIPP() { }
    ~SIPP() { }
    Path find_path(Agent agent, const Map &map, const std::vector<std::vector<int> > &cat, std::list<Constraint> cons = {});

private:
    Agent agent;
    void find_successors(const Node curNode, const Map &map, std::list<Node> &succs);
    void add_open(Node newNode, const std::vector<std::vector<int> > &cat);
    Node find_min(int size);
    int  count_h_value(int i, int j, int goal_i, int goal_j);
    void reconstruct_path(Node curNode);
    void make_constraints(std::list<Constraint> &cons);
    void clear();

    unsigned int openSize;
    std::unordered_multimap<int, Node> close;
    std::vector<std::list<Node>> open;
    Path path;
    std::map<Move, std::vector<Move>> constraints;//stores sets of constraints associated with moves
    std::map<std::pair<int,int>, std::vector<std::pair<int,int>>> collision_intervals;//stores sets of collision intervals associated with cells
};

#endif // SIPP_H
