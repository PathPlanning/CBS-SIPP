#include "cbs.h"

void CBS::init_root(const Map &map, const Task &task)
{
    CBS_Node root;
    Path path;
    cat.resize(map.height);
    for(int i = 0; i < map.height; i++)
        cat[i].resize(map.width, 0);
    make_cat(std::vector<Path>{});
    for(int i = 0; i < task.get_agents_size(); i++)
    {
        Agent agent = task.get_agent(i);
        path = planner.find_path(agent, map, cat);
        update_cat(path, true);
        root.paths.push_back(path);
        root.cost += path.cost;
    }
    root.parent = nullptr;
    root.cons_num = 0;
    tree.add_node(root);

}

void CBS::make_cat(const std::vector<Path> &paths)
{
    for(int i = 0; i < cat.size(); i++)
        for(int j = 0; j < cat[i].size(); j++)
            cat[i][j] = 0;
    for(auto path : paths)
        for(auto node : path.nodes)
            cat[node.i][node.j]++;
}

void CBS::update_cat(Path path, bool inc)
{
    for(auto node : path.nodes)
    {
        if(inc)
            cat[node.i][node.j]++;
        else
            cat[node.i][node.j]--;
    }
}

Solution CBS::find_solution(const Map &map, const Task &task)
{
    CBS_Node node;
    auto t = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_spent;
    int k;
    do
    {
        k++;
        auto parent = tree.get_front();
        node = *parent;
        tree.pop_front();
        auto paths = get_paths(&node, task.get_agents_size());
        int conflicts_num = count_conflicts(paths);
        Conflict conflict = check_conflicts(paths);
        if(conflict.agent1 == -1)
            break; //i.e. no conflicts => solution found
        make_cat(paths);

        std::list<Constraint> constraints = get_constraints(&node, conflict.agent1);
        constraints.push_back(conflict.get_constraint(1));
        Path left_old_path = paths.at(conflict.agent1);
        update_cat(left_old_path, false);
        Path path = planner.find_path(task.get_agent(conflict.agent1), map, cat, constraints);
        update_cat(left_old_path, true);
        paths.at(conflict.agent1) = path;
        conflicts_num = count_conflicts(paths);
        paths.at(conflict.agent1) = left_old_path;
        CBS_Node left({path}, parent, conflict.get_constraint(1), node.cost + path.cost - get_cost(node, conflict.agent1), node.cons_num + 1, conflicts_num);
        if(path.cost >= 0)
            tree.add_node(left);

        constraints = get_constraints(&node, conflict.agent2);
        constraints.push_back(conflict.get_constraint(2));
        update_cat(paths.at(conflict.agent2), false);
        path = planner.find_path(task.get_agent(conflict.agent2), map, cat, constraints);
        paths.at(conflict.agent2) = path;
        conflicts_num = count_conflicts(paths);
        CBS_Node right({path}, parent, conflict.get_constraint(2), node.cost + path.cost - get_cost(node, conflict.agent2), node.cons_num + 1, conflicts_num);
        if(path.cost >= 0)
            tree.add_node(right);

        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
        if(time_spent.count() > CN_TIMELIMIT)
            break;
    }
    while(tree.get_open_size() > 0);
    solution.paths = get_paths(&node, task.get_agents_size());
    solution.flowtime = node.cost;
    for(auto path:solution.paths)
        solution.makespan = std::max(solution.makespan, path.cost);
    time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.time = time_spent;
    return solution;
}

std::list<Constraint> CBS::get_constraints(CBS_Node *node, int agent_id)
{
    CBS_Node* curNode = node;
    std::list<Constraint> constraints(0);
    while(curNode->parent != nullptr)
    {
        if(curNode->constraint.agent == agent_id)
            constraints.push_back(curNode->constraint);
        curNode = curNode->parent;
    }
    return constraints;
}
int CBS::count_conflicts(std::vector<Path> &paths)
{
    int max_t(0);
    int conflicts_num(0);
    for(int i = 0; i < paths.size(); i++)
    {
        if(paths[i].cost > max_t)
            max_t = paths[i].cost;
        for(int j = 1; j < paths[i].nodes.size(); j++)
            paths[i].nodes[j].parent = &paths[i].nodes[j - 1];
    }

    for(int i = 0; i <= max_t; i++)
        for(int j = 0; j < paths.size(); j++)
        {
            Node a;
            if(paths[j].nodes.size() > i)
                a = paths[j].nodes[i];
            else
                a = paths[j].nodes.back();
            for(int k = j + 1; k < paths.size(); k++)
            {
                Node b;
                if(paths[k].nodes.size() > i)
                    b = paths[k].nodes[i];
                else
                    b = paths[k].nodes.back();
                if(i != 0 && i < paths[j].nodes.size() && i < paths[k].nodes.size())
                    if(a.parent->i == b.i && a.parent->j == b.j && a.i == b.parent->i && a.j == b.parent->j)
                        conflicts_num++;
                if(a.i == b.i && a.j == b.j)
                    conflicts_num++;
            }
        }
    return conflicts_num;
}
Conflict CBS::check_conflicts(std::vector<Path> &paths)
{
    double max_t(0);
    for(int i = 0; i < paths.size(); i++)
    {
        if(paths[i].cost > max_t)
            max_t = paths[i].cost;
        for(int j = 1; j < paths[i].nodes.size(); j++)
            paths[i].nodes[j].parent = &paths[i].nodes[j - 1];
    }

    for(int i = 0; i <= max_t; i++)
        for(int j = 0; j < paths.size(); j++)
        {
            Node a;
            if(paths[j].nodes.size() > i)
                a = paths[j].nodes[i];
            else
                a = paths[j].nodes.back();
            for(int k = j + 1; k < paths.size(); k++)
            {
                Node b;
                if(paths[k].nodes.size() > i)
                    b = paths[k].nodes[i];
                else
                    b = paths[k].nodes.back();
                if(i != 0 && i < paths[j].nodes.size() && i < paths[k].nodes.size())
                    if(a.parent->i == b.i && a.parent->j == b.j && a.i == b.parent->i && a.j == b.parent->j)
                    {
                        return Conflict(j, k, Move(i - 1, a.parent->i, a.parent->j, a.i, a.j), Move(i - 1, b.parent->i, b.parent->j, b.i, b.j), i - 1);
                    }
                if(a.i == b.i && a.j == b.j)
                {
                    return Conflict(j, k, Move(i, a.i, a.j, a.i, a.j), Move(i, b.i, b.j, b.i, b.j), i);
                }
            }
        }
    return Conflict();
}

double CBS::get_cost(CBS_Node node, int agent_id)
{
    while(node.parent != nullptr)
    {
        if(node.paths.begin()->agentID == agent_id)
            return node.paths.begin()->cost;
        node = *node.parent;
    }
    return node.paths.at(agent_id).cost;
}

std::vector<Path> CBS::get_paths(CBS_Node *node, int agents_size)
{
    CBS_Node* curNode = node;
    std::vector<Path> paths(agents_size);
    while(curNode->parent != nullptr)
    {
        if(paths.at(curNode->paths.begin()->agentID).cost < 0)
            paths.at(curNode->paths.begin()->agentID) = *curNode->paths.begin();
        curNode = curNode->parent;
    }
    for(int i = 0; i < agents_size; i++)
    {
        if(paths.at(i).cost < 0)
            paths.at(i) = curNode->paths.at(i);
    }
    return paths;
}
