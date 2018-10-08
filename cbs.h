#ifndef CBS_H
#define CBS_H
#include "structs.h"
#include "map.h"
#include "task.h"
#include "sipp.h"
#include <chrono>

class CBS
{
public:
    CBS() {}
    void make_cat(const std::vector<Path> &paths);
    void update_cat(Path path, bool inc);
    void init_root(const Map &map, const Task &task);
    Solution find_solution(const Map &map, const Task &task);
    std::list<Constraint> get_constraints(CBS_Node *node, int agent_id);
    Conflict check_conflicts(std::vector<Path> &paths);
    int count_conflicts(std::vector<Path> &paths);
    double get_cost(CBS_Node node, int agent_id);
    std::vector<Path> get_paths(CBS_Node *node, int agents_size);
    CBS_Tree tree;
    SIPP planner;
    Solution solution;
    std::vector<std::vector<int>> cat;//collision avoidance table
};

#endif // CBS_H
