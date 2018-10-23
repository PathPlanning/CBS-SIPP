#include "cbs.h"

void CBS::init_root(const Map &map, const Task &task)
{
    CBS_Node root;
    Path path;
    h_values.init(map.height, map.width, task.get_agents_size(), CN_K);
    for(int i = 0; i < task.get_agents_size(); i++)
    {
        Agent agent = task.get_agent(i);
        h_values.count(map, agent);
        path = planner.find_path(agent, map, {}, &h_values);
        root.paths.push_back(path);
        root.cost += path.cost;
    }
    root.parent = nullptr;
    root.cons_num = 0;
    tree.add_node(root);
}

Constraint CBS::get_constraint(int agent, Move move1, Move move2)
{
    double begin(move1.t1), startTimeA(move1.t1), endTimeA(move1.t2), startTimeB(move2.t1), endTimeB(move2.t2);
    Vector2D A(move1.i1, move1.j1);
    Vector2D B(move2.i1, move2.j1);
    Vector2D VA((move1.i2 - move1.i1)/(move1.t2 - move1.t1), (move1.j2 - move1.j1)/(move1.t2 - move1.t1));
    Vector2D VB((move2.i2 - move2.i1)/(move2.t2 - move2.t1), (move2.j2 - move2.j1)/(move2.t2 - move2.t1));
    while(startTimeA < endTimeB)
    {
        A = Vector2D(move1.i1, move1.j1);
        startTimeA = move1.t1;
        endTimeA = move1.t2;
        if(startTimeB > startTimeA)
        {
            A += VA*(startTimeB-startTimeA);
            startTimeA = startTimeB;
        }
        else if(startTimeB < startTimeA)
        {
            B += VB*(startTimeA - startTimeB);
            startTimeB = startTimeA;
        }

        double r(2*CN_AGENT_SIZE);
        Vector2D w(B - A);
        double c(w*w - r*r);
        if(c < 0)
        {
            if(move2.t2 == CN_INFINITY)
                return Constraint(agent, begin, CN_INFINITY, move1.i1, move1.j1, move1.i2, move1.j2);
            move1.t1 += 0.1;
            move1.t2 += 0.1;
            continue;
        }

        Vector2D v(VA - VB);
        double a(v*v);
        double b(w*v);
        double dscr(b*b - a*c);
        if(dscr <= 0)
        {
            break;
        }

        double ctime = (b - sqrt(dscr))/a;
        if(ctime > -CN_EPSILON && ctime < std::min(endTimeB,endTimeA) - startTimeA + CN_EPSILON)
        {
            if(move2.t2 == CN_INFINITY)
                return Constraint(agent, begin, CN_INFINITY, move1.i1, move1.j1, move1.i2, move1.j2);
            move1.t1 += 0.1;
            move1.t2 += 0.1;
            continue;
        }
        else
        {
            break;
        }
    }
    return Constraint(agent, begin, move1.t1, move1.i1, move1.j1, move1.i2, move1.j2);
}


Solution CBS::find_solution(const Map &map, const Task &task)
{
    auto t = std::chrono::high_resolution_clock::now();
    this->init_root(map, task);
    CBS_Node node;
    std::chrono::duration<double> time_spent;
    do
    {
        auto parent = tree.get_front();
        node = *parent;
        tree.pop_front();
        auto paths = get_paths(&node, task.get_agents_size());
        Conflict conflict = check_conflicts(paths);
        if(conflict.agent1 == -1)
            break; //i.e. no conflicts => solution found

        std::list<Constraint> constraints = get_constraints(&node, conflict.agent1);
        Constraint constraint(get_constraint(conflict.agent1, conflict.move1, conflict.move2));
        constraints.push_back(constraint);
        Path path = planner.find_path(task.get_agent(conflict.agent1), map, constraints, &h_values);
        CBS_Node left({path}, parent, constraint, node.cost + path.cost - get_cost(node, conflict.agent1), node.cons_num + 1);
        if(path.cost >= 0)
            tree.add_node(left);


        constraints = get_constraints(&node, conflict.agent2);
        constraint = get_constraint(conflict.agent2, conflict.move2, conflict.move1);
        constraints.push_back(constraint);
        path = planner.find_path(task.get_agent(conflict.agent2), map, constraints, &h_values);
        CBS_Node right({path}, parent, constraint, node.cost + path.cost - get_cost(node, conflict.agent2), node.cons_num + 1);
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
        solution.makespan = (solution.makespan > path.cost) ? solution.makespan : path.cost;
    solution.time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t);
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

Conflict CBS::check_conflicts(std::vector<Path> &paths)
{
    Position cur, check, conf;
    std::vector<std::vector<Position>> positions;
    positions.resize(paths.size());
    for(int i = 0; i < paths.size(); i++)
    {
        if(paths[i].cost < 0)
            continue;
        positions[i].resize(0);
        int k = 0;
        double part = 1;
        for(int j = 1; j < paths[i].nodes.size(); j++)
        {
            cur = paths[i].nodes[j];
            check = paths[i].nodes[j-1];
            int di = cur.i - check.i;
            int dj = cur.j - check.j;
            double dist = (cur.t - check.t)*10;
            int steps = (cur.t - check.t)*10;
            if(dist - steps + part >= 1)
            {
                steps++;
                part = dist - steps;
            }
            else
                part += dist - steps;
            double stepi = double(di)/dist;
            double stepj = double(dj)/dist;
            double curt = double(k)*0.1;
            double curi = check.i + (curt - check.t)*di/(cur.t - check.t);
            double curj = check.j + (curt - check.t)*dj/(cur.t - check.t);
            conf.i = curi;
            conf.j = curj;
            conf.t = curt;
            if(curt <= cur.t)
            {
                positions[i].push_back(conf);
                k++;
            }
            while(curt <= cur.t)
            {
                if(curt + 0.1 > cur.t)
                    break;
                curi += stepi;
                curj += stepj;
                curt += 0.1;
                conf.i = curi;
                conf.j = curj;
                conf.t = curt;
                positions[i].push_back(conf);
                k++;
            }
        }
        if(double(k - 1)*0.1 < paths[i].nodes.back().g)
        {
            conf.i = paths[i].nodes.back().i;
            conf.j = paths[i].nodes.back().j;
            conf.t = paths[i].nodes.back().g;
            positions[i].push_back(conf);
        }
    }
    int max = 0;
    for(int i = 0; i < positions.size(); i++)
        if(positions[i].size() > max)
            max = positions[i].size();
    for(int i = 0; i < paths.size(); i++)
    {
        for(int k = 0; k < max; k++)
        {
            for(int j = i + 1; j < paths.size(); j++)
            {
                Position a, b;
                if(positions[i].size() > k)
                    a = positions[i][k];
                else
                    a = positions[i].back();
                if(positions[j].size() > k)
                    b = positions[j][k];
                else
                    b = positions[j].back();
                if(sqrt((a.i - b.i)*(a.i - b.i) + (a.j - b.j)*(a.j - b.j)) + CN_EPSILON < 2*CN_AGENT_SIZE)
                {

                    //std::cout<<a.i<<" "<<a.j<<" "<<b.i<<" "<<b.j<<" "<<k<<" collision\n";
                    //std::cout<<positions[i].size()<<" "<<positions[j].size()<<" "<<k<<" sizes\n";
                    //if(positions[i].size() == 86 && a.i==4 && a.j==8)
                    //for(int h=0; h<positions[i].size(); h++)
                    //    std::cout<<positions[i][h].i<<" "<<positions[i][h].j<<" "<<positions[i][h].t<<" pos\n";
                    Move move1, move2;
                    for(int p = 1; p < paths[i].nodes.size(); p++)
                        if(paths[i].nodes[p-1].g < a.t + CN_EPSILON && paths[i].nodes[p].g > a.t - CN_EPSILON)
                            move1 = Move(paths[i].nodes[p-1], paths[i].nodes[p]);
                    for(int p = 1; p < paths[j].nodes.size(); p++)
                        if(paths[j].nodes[p-1].g < b.t + CN_EPSILON && paths[j].nodes[p].g > b.t - CN_EPSILON)
                            move2 = Move(paths[j].nodes[p-1], paths[j].nodes[p]);
                    if(a.t*10 + CN_EPSILON < k)
                    {
                        move1 = Move(paths[i].nodes.back(), paths[i].nodes.back());
                        move1.t2 = CN_INFINITY;
                    }
                    if(b.t*10 + CN_EPSILON < k)
                    {
                        move2 = Move(paths[j].nodes.back(), paths[j].nodes.back());
                        move2.t2 = CN_INFINITY;
                    }
                    return Conflict(i, j, move1, move2, a.t > b.t ? a.t : b.t);
                }
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
