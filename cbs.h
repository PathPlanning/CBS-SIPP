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
    Solution find_solution(const Map &map, const Task &task);
private:
    void make_cat(const std::vector<Path> &paths);
    void update_cat(Path path, bool inc);
    bool init_root(const Map &map, const Task &task);
    std::list<Constraint> get_constraints(CBS_Node *node, int agent_id = -1);
    std::vector<Conflict> get_all_conflicts(std::vector<Path> &paths);
    Conflict check_conflicts(std::vector<Path> &paths);
    Conflict check_paths(Path pathA, Path pathB);
    int count_conflicts(std::vector<Path> &paths);
    double get_cost(CBS_Node node, int agent_id);
    std::vector<Path> get_paths(CBS_Node *node, int agents_size);
    CBS_Tree tree;
    SIPP planner;
    Heuristic h_values;
    Solution solution;
    int initial_cost;
    std::vector<std::vector<int>> cat;//collision avoidance table
    std::vector<std::pair<int, int>> conflicting_pairs;
    std::vector<int> conflicting_agents;
};

#endif // CBS_H
