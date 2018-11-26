#include "sipp.h"

void SIPP::clear()
{
    open.clear();
    close.clear();
    collision_intervals.clear();
    constraints.clear();
    openSize = 0;
    path.cost = -1;
}

int SIPP::count_h_value(int i, int j, int goal_i, int goal_j)
{
    return abs(i - goal_i) + abs(j - goal_j);
}

void SIPP::find_successors(const Node curNode, const Map &map, std::list<Node> &succs)
{
    Node newNode;
    for(auto move : map.get_valid_moves(curNode.i, curNode.j))
    {
        newNode.i = curNode.i + move.i;
        newNode.j = curNode.j + move.j;
        newNode.g = curNode.g + 1;
        std::vector<std::pair<int, int>> intervals(0);
        auto colls_it = collision_intervals.find(std::make_pair(newNode.i, newNode.j));
        if(colls_it != collision_intervals.end())
        {
            std::pair<int, int> interval = {0, CN_INFINITY};
            for(unsigned int i = 0; i < colls_it->second.size(); i++)
            {
                interval.second = colls_it->second[i].first - 1;
                intervals.push_back(interval);
                interval.first = colls_it->second[i].second + 1;
            }
            interval.second = CN_INFINITY;
            intervals.push_back(interval);
        }
        else
            intervals.push_back({newNode.g, CN_INFINITY});
        auto cons_it = constraints.find(Move(curNode.g, curNode.i, curNode.j, newNode.i, newNode.j));
        for(auto interval: intervals)
        {
            if(interval.second < newNode.g)
                continue;
            if(interval.first > newNode.g)
                newNode.g = interval.first;
            if(cons_it != constraints.end())
                for(unsigned int i = 0; i < cons_it->second.size(); i++)
                    if(newNode.g - 1 == cons_it->second[i].t)
                        newNode.g++;
            newNode.interval = interval;
            if(newNode.g > curNode.interval.second + 1 || newNode.g > newNode.interval.second)
                continue;
            newNode.f = newNode.g + h_values->get_value(newNode.i, newNode.j, agent.id);
            succs.push_back(newNode);
        }
    }
}

Node SIPP::find_min(int size)
{
    Node min;
    min.f = CN_INFINITY;
    for(int i = 0; i < size; i++)
        if(!open[i].empty())
            if(open[i].begin()->f < min.f || (open[i].begin()->f == min.f && open[i].begin()->g > min.g))
                min = *open[i].begin();
    return min;
}

void SIPP::add_open(Node newNode, const std::vector<std::vector<int>> &cat)
{
    std::list<Node>::iterator iter, pos;
    bool posFound = false;
    pos = open[newNode.i].end();
    if (open[newNode.i].size() == 0)
    {
        open[newNode.i].push_back(newNode);
        openSize++;
        return;
    }
    for(iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
    {
        if (iter->f >= newNode.f && !posFound)
        {
            if (iter->f == newNode.f)
            {
                if (cat[newNode.i][newNode.j] < cat[iter->i][iter->j] || (cat[newNode.i][newNode.j] == cat[iter->i][iter->j] && newNode.g >= iter->g))
                {
                    pos = iter;
                    posFound = true;
                }
            }
            else
            {
                pos = iter;
                posFound = true;
            }
        }
        if (iter->j == newNode.j && iter->interval.second == newNode.interval.second)
        {
            if(newNode.f >= iter->f)
                return;
            if(pos == iter)
            {
                iter->f = newNode.f;
                iter->g = newNode.g;
                iter->interval = newNode.interval;
                iter->parent = newNode.parent;
                return;
            }
            open[newNode.i].erase(iter);
            openSize--;
            break;
        }
    }
    openSize++;
    open[newNode.i].insert(pos, newNode);
}

void SIPP::reconstruct_path(Node curNode)
{
    path.nodes.clear();
    if(curNode.parent != nullptr)
    do
    {
        path.nodes.insert(path.nodes.begin(), curNode);
        curNode = *curNode.parent;
    }
    while(curNode.parent != nullptr);
    path.nodes.insert(path.nodes.begin(), curNode);
    for(int i = 0; i < path.nodes.size(); i++)
    {
        int j = i + 1;
        if(j == path.nodes.size())
            break;
        if(path.nodes[j].g - path.nodes[i].g != 1)
        {
            Node add = path.nodes[i];
            add.g++;
            add.f++;
            path.nodes.emplace(path.nodes.begin() + j, add);
            i--;
        }
    }
}

void SIPP::make_constraints(std::list<Constraint> &cons)
{
    for(auto con : cons)
    {
        if(con.i1 == con.i2 && con.j1 == con.j2)
        {
            std::pair<int, int> interval = {con.t,con.t};
            std::vector<std::pair<int,int>> intervals(0);
            if(collision_intervals.count(std::make_pair(con.i1, con.j1)) == 0)
                collision_intervals.insert({std::make_pair(con.i1, con.j1), {interval}});
            else
            {
                intervals = collision_intervals.at(std::make_pair(con.i1, con.j1));
                for(unsigned int i = 0; i < intervals.size(); i++)
                {
                    if(intervals[i].first > interval.first + 1)
                    {
                        intervals.insert(intervals.begin() + i, interval);
                        break;
                    }
                    else if(intervals[i].first == interval.first + 1)
                    {
                        intervals[i].first = interval.first;
                        break;
                    }
                    else if(intervals[i].second == interval.second - 1)
                    {
                        intervals[i].second = interval.second;
                        break;
                    }
                    else if(i + 1 == intervals.size())
                    {
                        intervals.push_back(interval);
                        break;
                    }
                }
                collision_intervals.at(std::make_pair(con.i1, con.j1)) = intervals;
            }
        }
        else
        {
            Move move(con);
            std::vector<Move> cons(0);
            if(constraints.count(move) == 0)
                constraints.insert({move, {move}});
            else
            {
                cons = constraints.at(move);
                bool inserted = false;
                for(unsigned int i = 0; i < cons.size(); i++)
                    if(cons[i].t > move.t)
                    {
                        cons.insert(cons.begin() + i, move);
                        inserted = true;
                        break;
                    }
                if(!inserted)
                    cons.push_back(move);
                constraints.at(move) = cons;
            }
        }
    }
}

Path SIPP::find_path(Agent agent, const Map &map, const std::vector<std::vector<int>> &cat, std::list<Constraint> cons, Heuristic *h_values)
{
    this->clear();
    this->h_values = h_values;
    this->agent = agent;
    make_constraints(cons);
    open.resize(map.height);
    Node curNode(agent.start_i, agent.start_j, 0, 0);
    if(collision_intervals.count(std::make_pair(curNode.i, curNode.j)) > 0)
    {
        auto intervals = collision_intervals.at(std::make_pair(curNode.i, curNode.j));
        curNode.interval = {0, intervals[0].first - 1};
    }
    else
        curNode.interval = {0, CN_INFINITY};

    bool pathFound = false;
    open[curNode.i].push_back(curNode);
    openSize++;
    while(openSize > 0)
    {
        curNode = find_min(map.height);
        open[curNode.i].pop_front();
        openSize--;
        close.insert({curNode.i * map.width + curNode.j, curNode});
        if(curNode.i == agent.goal_i && curNode.j == agent.goal_j && curNode.interval.second == CN_INFINITY)
        {
            pathFound = true;
            break;
        }
        std::list<Node> succs;
        succs.clear();
        find_successors(curNode, map, succs);
        std::list<Node>::iterator it = succs.begin();
        auto parent = &(close.find(curNode.i * map.width + curNode.j)->second);
        while(it != succs.end())
        {
            bool has = false;
            it->parent = parent;
            auto range = close.equal_range(it->i * map.width + it->j);
            for(auto i = range.first; i != range.second; i++)
                if(i->second.interval.first <= it->interval.first && i->second.interval.second >= it->interval.second)
                {
                    has = true;
                    break;
                }
            if(!has)
                add_open(*it, cat);
            it++;
        }
    }
    if (pathFound)
    {
        reconstruct_path(curNode);
        path.cost = curNode.g;
    }
    path.expanded = close.size();
    path.agentID = agent.id;
    return path;
}
