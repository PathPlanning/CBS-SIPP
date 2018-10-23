#ifndef CBS_H
#define CBS_H
#include <chrono>
#include "structs.h"
#include "map.h"
#include "task.h"
#include "sipp.h"
#include "heuristic.h"

class CBS
{
public:
    CBS() {}
    Solution find_solution(const Map &map, const Task &task);
private:
    void init_root(const Map &map, const Task &task);
    std::list<Constraint> get_constraints(CBS_Node *node, int agent_id);
    Conflict check_conflicts(std::vector<Path> &paths);
    //int count_conflicts(std::vector<Path> &paths);
    Constraint get_constraint(int agent, Move move1, Move move2);
    double get_cost(CBS_Node node, int agent_id);
    std::vector<Path> get_paths(CBS_Node *node, int agents_size);
    CBS_Tree tree;
    SIPP planner;
    Solution solution;
    Heuristic h_values;
};

#endif // CBS_H
