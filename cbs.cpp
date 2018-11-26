#include "cbs.h"

bool CBS::init_root(const Map &map, const Task &task)
{
    CBS_Node root;
    Path path;
    initial_cost = 0;
    cat.resize(map.height);
    for(int i = 0; i < map.height; i++)
        cat[i].resize(map.width, 0);
    make_cat(std::vector<Path>{});
    for(int i = 0; i < task.get_agents_size(); i++)
    {
        Agent agent = task.get_agent(i);
        h_values.count(map, agent);
        path = planner.find_path(agent, map, cat, {}, &h_values);
        if(path.cost < 0)
            return false;
        initial_cost += path.cost;
        update_cat(path, true);
        root.paths.push_back(path);
        root.cost += path.cost;
    }
    root.parent = nullptr;
    root.cons_num = 0;
    root.id = 0;
    tree.add_node(root);
    return true;

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
    auto t = std::chrono::high_resolution_clock::now();
    h_values.init(map.height, map.width, task.get_agents_size());
    if(!this->init_root(map, task))
        return solution;
    CBS_Node node;
    solution.init_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    std::chrono::duration<double> time_spent;
    int k;
    int expanded(0);
    double ll_expands(0);
    solution.ll_expanded = 0;
    solution.ll_searches = 0;
    std::vector<Conflict> conflicts;
    Conflict conflict;
    bool solution_found = false;
    do
    {
        k++;
        auto parent = tree.get_front();
        node = *parent;
        tree.pop_front();
        auto paths = get_paths(&node, task.get_agents_size());
        if(!CN_CARDINAL)
            node.look_for_cardinal = false;
        if(node.look_for_cardinal)
            conflicts = get_all_conflicts(paths);
        else
            conflict = check_conflicts(paths);
        if((node.look_for_cardinal && conflicts.empty()) || (!node.look_for_cardinal && conflict.agent1 < 0))
            break; //i.e. no conflicts => solution found
        expanded++;
        if(CN_USE_CAT)
            make_cat(paths);
        if(!node.look_for_cardinal && CN_HISTORY)
        {
            if(std::find(conflicting_agents.begin(), conflicting_agents.end(), conflict.agent1) == conflicting_agents.end())
                conflicting_agents.push_back(conflict.agent1);
            if(std::find(conflicting_agents.begin(), conflicting_agents.end(), conflict.agent2) == conflicting_agents.end())
                conflicting_agents.push_back(conflict.agent2);
            if(std::find(conflicting_pairs.begin(), conflicting_pairs.end(), std::make_pair(conflict.agent1, conflict.agent2)) == conflicting_pairs.end())
                conflicting_pairs.push_back({conflict.agent1, conflict.agent2});
        }
        bool cardinal_found = false;
        Conflict semi_cardinal;
        if(node.look_for_cardinal)
        {
            for(auto conflict:conflicts)
            {
                std::list<Constraint> constraints = get_constraints(&node, conflict.agent1);
                constraints.push_back(conflict.get_constraint(1));
                Path left_old_path = paths.at(conflict.agent1);
                if(CN_USE_CAT)
                    update_cat(left_old_path, false);
                Path pathA = planner.find_path(task.get_agent(conflict.agent1), map, cat, constraints, &h_values);
                solution.ll_searches++;
                ll_expands += pathA.expanded;
                constraints = get_constraints(&node, conflict.agent2);
                constraints.push_back(conflict.get_constraint(2));
                if(CN_USE_CAT)
                {
                    update_cat(left_old_path, true);
                    update_cat(paths.at(conflict.agent2), false);
                }
                Path pathB = planner.find_path(task.get_agent(conflict.agent2), map, cat, constraints, &h_values);
                solution.ll_searches++;
                ll_expands += pathB.expanded;
                if(pathA.cost > get_cost(node, conflict.agent1) && pathB.cost > get_cost(node, conflict.agent2))
                {

                    CBS_Node left({pathA}, parent, conflict.get_constraint(1), node.cost + pathA.cost - get_cost(node, conflict.agent1), node.cons_num + 1, conflicts.size(), node.look_for_cardinal);
                    CBS_Node right({pathB}, parent, conflict.get_constraint(2), node.cost + pathB.cost - get_cost(node, conflict.agent2), node.cons_num + 1, conflicts.size(), node.look_for_cardinal);
                    left.id = node.id*10+1;
                    right.id = node.id*10+2;
                    paths.at(conflict.agent1) = pathA;
                    auto confs = get_all_conflicts(paths);
                    left.conflicts_num = confs.size();
                    paths.at(conflict.agent1) = left_old_path;
                    paths.at(conflict.agent2) = pathB;
                    confs = get_all_conflicts(paths);
                    right.conflicts_num = confs.size();
                    tree.add_node(left);
                    tree.add_node(right);
                    cardinal_found = true;
                    break;
                }
                if(pathA.cost > get_cost(node, conflict.agent1) || pathB.cost > get_cost(node, conflict.agent2))
                    semi_cardinal = conflict;
            }
        }
        if(solution_found)
            break;
        if(!cardinal_found || !node.look_for_cardinal)
        {
            if(node.look_for_cardinal)
            {
                conflict = conflicts.at(0);
                if(semi_cardinal.agent1 >= 0)
                    conflict = semi_cardinal;
                else if(CN_STOP_CARDINAL)
                    node.look_for_cardinal = false;
            }
            std::list<Constraint> constraints = get_constraints(&node, conflict.agent1);
            constraints.push_back(conflict.get_constraint(1));
            Path left_old_path = paths.at(conflict.agent1);
            if(CN_USE_CAT)
                update_cat(left_old_path, false);
            Path pathA = planner.find_path(task.get_agent(conflict.agent1), map, cat, constraints, &h_values);
            solution.ll_searches++;
            ll_expands +=pathA.expanded;
            constraints = get_constraints(&node, conflict.agent2);
            constraints.push_back(conflict.get_constraint(2));
            if(CN_USE_CAT)
            {
                update_cat(left_old_path, true);
                update_cat(paths.at(conflict.agent2), false);
            }
            Path pathB = planner.find_path(task.get_agent(conflict.agent2), map, cat, constraints, &h_values);
            solution.ll_searches++;
            ll_expands +=pathB.expanded;
            CBS_Node left({pathA}, parent, conflict.get_constraint(1), node.cost + pathA.cost - get_cost(node, conflict.agent1), node.cons_num + 1, conflicts.size(), node.look_for_cardinal);
            CBS_Node right({pathB}, parent, conflict.get_constraint(2), node.cost + pathB.cost - get_cost(node, conflict.agent2), node.cons_num + 1, conflicts.size(), node.look_for_cardinal);

            left.id = node.id*10+1;
            right.id = node.id*10+2;
            if(pathA.cost >= 0)
            {
                paths.at(conflict.agent1) = pathA;
                conflicts = get_all_conflicts(paths);
                left.conflicts_num = conflicts.size();
                tree.add_node(left);
            }
            if(pathB.cost >= 0)
            {
                paths.at(conflict.agent1) = left_old_path;
                paths.at(conflict.agent2) = pathB;
                conflicts = get_all_conflicts(paths);
                right.conflicts_num = conflicts.size();
                tree.add_node(right);
            }
        }
        time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
        if(time_spent.count() > CN_TIMELIMIT)
            break;
    }
    while(tree.get_open_size() > 0 && !solution_found);
    solution.paths = get_paths(&node, task.get_agents_size());
    solution.flowtime = node.cost;
    for(auto path:solution.paths)
        solution.makespan = std::max(solution.makespan, path.cost);
    time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
    solution.time = time_spent;
    solution.expanded = expanded;
    solution.ll_expanded = ll_expands/solution.ll_searches;
    solution.initial_cost = initial_cost;
    return solution;
}

std::list<Constraint> CBS::get_constraints(CBS_Node *node, int agent_id)
{
    CBS_Node* curNode = node;
    std::list<Constraint> constraints(0);
    while(curNode->parent != nullptr)
    {
        if(agent_id < 0 || curNode->constraint.agent == agent_id)
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

Conflict CBS::check_paths(Path pathA, Path pathB)
{
    int i(0), j(0);
    while(i < pathA.nodes.size() - 1 || j < pathB.nodes.size())
    {
        if(i == pathA.nodes.size() - 1)
        {
            if(pathA.nodes[i].i == pathB.nodes[j].i && pathA.nodes[i].j == pathB.nodes[j].j)
                return Conflict(pathA.agentID, pathB.agentID,
                                Move(j, pathA.nodes[i].i, pathA.nodes[i].j, pathA.nodes[i].i, pathA.nodes[i].j),
                                Move(j, pathB.nodes[j].i, pathB.nodes[j].j, pathB.nodes[j].i, pathB.nodes[j].j), j);
            j++;
        }
        else if(j == pathB.nodes.size() - 1)
        {
            if(pathA.nodes[i].i == pathB.nodes[j].i && pathA.nodes[i].j == pathB.nodes[j].j)
                return Conflict(pathA.agentID, pathB.agentID,
                                Move(i, pathA.nodes[i].i, pathA.nodes[i].j, pathA.nodes[i].i, pathA.nodes[i].j),
                                Move(i, pathB.nodes[j].i, pathB.nodes[j].j, pathB.nodes[j].i, pathB.nodes[j].j), i);
            i++;
        }
        else
        {
            if(pathA.nodes[i].i == pathB.nodes[j].i && pathA.nodes[i].j == pathB.nodes[j].j)
                return Conflict(pathA.agentID, pathB.agentID,
                                Move(i, pathA.nodes[i].i, pathA.nodes[i].j, pathA.nodes[i].i, pathA.nodes[i].j),
                                Move(j, pathB.nodes[j].i, pathB.nodes[j].j, pathB.nodes[j].i, pathB.nodes[j].j), i);
            if(pathA.nodes[i].i == pathB.nodes[j+1].i && pathA.nodes[i].j == pathB.nodes[j+1].j
                    && pathA.nodes[i+1].i == pathB.nodes[j].i && pathA.nodes[i+1].j == pathB.nodes[j].j)
                return Conflict(pathA.agentID, pathB.agentID,
                                Move(i, pathA.nodes[i].i, pathA.nodes[i].j, pathA.nodes[i+1].i, pathA.nodes[i+1].j),
                                Move(j, pathB.nodes[j].i, pathB.nodes[j].j, pathB.nodes[j+1].i, pathB.nodes[j+1].j), i);
            i++;
            j++;
        }
    }
    return Conflict();
}

Conflict CBS::check_conflicts(std::vector<Path> &paths)
{
    Conflict conflict;
    //first of all check the pairs of agents that already have collisions
    for(int i = 0; i < conflicting_pairs.size(); i++)
    {
        conflict = check_paths(paths[conflicting_pairs[i].first], paths[conflicting_pairs[i].second]);
        if(conflict.agent1 >= 0)
            return conflict;
    }
    //check other possible combinations of agents that already have conflicts
    for(int i = 0; i < conflicting_agents.size(); i++)
        for(int j = i + 1; j < conflicting_agents.size(); j++)
            if(std::find(conflicting_pairs.begin(), conflicting_pairs.end(), std::make_pair(conflicting_agents[i], conflicting_agents[j])) == conflicting_pairs.end())
            {
                conflict = check_paths(paths[conflicting_agents[i]], paths[conflicting_agents[j]]);
                if(conflict.agent1 >= 0)
                    return conflict;
            }
    //check all other pairs of agents
    for(int i = 0; i < paths.size(); i++)
        for(int j = i + 1; j < paths.size(); j++)
            if(std::find(conflicting_agents.begin(), conflicting_agents.end(), i) == conflicting_agents.end()
               || std::find(conflicting_agents.begin(), conflicting_agents.end(), j) == conflicting_agents.end())
            {
                conflict = check_paths(paths[i], paths[j]);
                if(conflict.agent1 >= 0)
                    return conflict;
            }
    //no conflicts were found
    return Conflict();
}

std::vector<Conflict> CBS::get_all_conflicts(std::vector<Path> &paths)
{
    std::vector<Conflict> conflicts;
    //check all agents
    for(int i = 0; i < paths.size(); i++)
        for(int j = i + 1; j < paths.size(); j++)
        {
            Conflict conflict = check_paths(paths[i], paths[j]);
            if(conflict.agent1 >= 0)
                conflicts.push_back(conflict);
        }
    return conflicts;
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
