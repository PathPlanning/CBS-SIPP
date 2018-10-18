#include "sipp.h"

void SIPP::clear()
{
    open.clear();
    close.clear();
    collision_intervals.clear();
    constraints.clear();
    openSize = 0;
    path.cost = -1;
    generate_moves(CN_K);

}

int SIPP::count_h_value(int i, int j, int goal_i, int goal_j)
{
    return sqrt(pow(i - goal_i,2) + pow(j - goal_j,2));
}

double SIPP::dist(const Node& a, const Node& b)
{
    return std::sqrt(pow(a.i - b.i, 2) + pow(a.j - b.j, 2));
}

void SIPP::generate_moves(int k)
{
    moves_2k.clear();
    if(k == 2)
        moves_2k = {{0,1}, {1,0}, {-1,0},  {0,-1}};
    else if(k == 3)
        moves_2k = {{0,1}, {1,0}, {-1,0},  {0,-1},  {1,1},  {1,-1}, {-1,-1}, {-1,1}};
    else if(k == 4)
        moves_2k = {{0,1}, {1,0}, {-1,0},  {0,-1},  {1,1},  {1,-1}, {-1,-1}, {-1,1},
                    {2,1}, {1,2}, {-2,-1}, {-1,-2}, {-2,1}, {-1,2}, {2,-1},  {1,-2}};
    else
        moves_2k = {{0,1}, {1,0}, {-1,0},  {0,-1},  {1,1},  {1,-1}, {-1,-1}, {-1,1},
                    {2,1}, {1,2}, {-2,-1}, {-1,-2}, {-2,1}, {-1,2}, {2,-1},  {1,-2},
                    {3,1}, {3,2}, {-3,-1}, {-3,-2}, {-3,1}, {-3,2}, {3,-1},  {3,-2},
                    {1,3}, {2,3}, {-1,-3}, {-2,-3}, {-1,3}, {-2,3}, {1,-3},  {2,-3}};
}

void SIPP::find_successors(Node curNode, const Map &map, std::list<Node> &succs)
{
    Node newNode;
    for(auto move : moves_2k)
    {
        newNode.i = curNode.i + move.first;
        newNode.j = curNode.j + move.second;
        if(map.cell_on_grid(newNode.i, newNode.j) && map.cell_is_traversable(newNode.i, newNode.j)
                && los.checkLine(newNode.i, newNode.j, curNode.i, curNode.j, map, CN_AGENT_SIZE))
        {
            newNode.g = curNode.g + dist(newNode, curNode);
            std::vector<std::pair<double, double>> intervals(0);
            auto colls_it = collision_intervals.find(std::make_pair(newNode.i, newNode.j));
            if(colls_it != collision_intervals.end())
            {
                std::pair<double, double> interval = {0, CN_INFINITY};
                for(unsigned int i = 0; i < colls_it->second.size(); i++)
                {
                    interval.second = colls_it->second[i].first;
                    intervals.push_back(interval);
                    interval.first = colls_it->second[i].second;
                }
                interval.second = CN_INFINITY;
                intervals.push_back(interval);
            }
            else
                intervals.push_back({newNode.g, CN_INFINITY});
            auto cons_it = constraints.find(Move(curNode.g, newNode.g, curNode.i, curNode.j, newNode.i, newNode.j));
            for(auto interval: intervals)
            {
                if(interval.second < newNode.g)
                    continue;
                if(interval.first > newNode.g)
                    newNode.g = interval.first;
                if(cons_it != constraints.end())
                    for(unsigned int i = 0; i < cons_it->second.size(); i++)
                    {
                        //std::cout<<curNode.i<<" "<<curNode.j<<" "<<cons_it->second[i].t1<<" "<<cons_it->second[i].t2<<" "<<curNode.g<<" sipp\n";
                        if(newNode.g - dist(newNode, curNode) + CN_EPSILON > cons_it->second[i].t1 && newNode.g - dist(newNode, curNode) < cons_it->second[i].t2)
                        {
                            newNode.g = cons_it->second[i].t2 + dist(curNode, newNode);
                            //std::cout<<newNode.g<<" upd\n";
                        }
                    }
                newNode.interval = interval;
                if(newNode.g - dist(newNode, curNode) > curNode.interval.second || newNode.g > newNode.interval.second)
                    continue;
                newNode.f = newNode.g + h_values->get_value(newNode.i, newNode.j, agent.id);
                succs.push_back(newNode);
            }
        }
    }
}

Node SIPP::find_min(int size)
{
    Node min;
    min.f = -1;
    for(int i = 0; i < size; i++)
    {
        if(!open[i].empty())
            if(open[i].begin()->f <= min.f || min.f == -1)
            {
                if (open[i].begin()->f == min.f)
                {
                    if (open[i].begin()->g >= min.g)
                        min = *open[i].begin();
                }
                else
                    min = *open[i].begin();
            }
    }
    return min;
}

void SIPP::add_open(Node newNode)
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
        if ((iter->f >= newNode.f) && (!posFound))
        {
            if (iter->f == newNode.f)
            {
                if (newNode.g <= iter->g)
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
        if(fabs(path.nodes[j].g - path.nodes[i].g - dist(path.nodes[j], path.nodes[i])) > CN_EPSILON)
        {
            Node add = path.nodes[i];
            //std::cout<<path.nodes[j].g<<" "<<path.nodes[i].g<<" "<<dist(path.nodes[j], path.nodes[i])<<"\n";
            add.g = path.nodes[j].g - dist(path.nodes[j], path.nodes[i]);
            path.nodes.emplace(path.nodes.begin() + j, add);
        }
    }
    Node goal = path.nodes.back();
    goal.g = CN_INFINITY;
    //path.nodes.push_back(goal);
}

void SIPP::make_constraints(std::list<Constraint> &cons)
{
    for(auto con : cons)
    {
        if(con.i1 == con.i2 && con.j1 == con.j2)
        {
            std::pair<double, double> interval = {con.t1, con.t2};
            std::vector<std::pair<double, double>> intervals(0);
            if(collision_intervals.count(std::make_pair(con.i1, con.j1)) == 0)
                collision_intervals.insert({std::make_pair(con.i1, con.j1), {interval}});
            else
            {
                intervals = collision_intervals.at(std::make_pair(con.i1, con.j1));
                bool inserted(false);
                for(unsigned int i = 0; i < intervals.size(); i++)
                {
                    if(inserted)
                        break;
                    if(interval.first < intervals[i].first + CN_EPSILON)
                    {
                        if(interval.second + CN_EPSILON > intervals[i].first)//a
                        {
                            intervals[i].first = interval.first;
                            if(interval.second + CN_EPSILON > intervals[i].second)//d
                                intervals[i].second = interval.second;
                            inserted = true;
                            if(i != 0)
                                if(intervals[i-1].second + CN_EPSILON > interval.first && intervals[i-1].second < interval.second + CN_EPSILON)
                                {
                                    intervals[i-1].second = interval.second;
                                    if(intervals[i-1].second < intervals[i].second + CN_EPSILON)
                                    {
                                        intervals[i-1].second = intervals[i].second;
                                        intervals.erase(intervals.begin() + i);
                                    }
                                    inserted = true;
                                }
                        }
                        else
                        {
                            if(i != 0)
                                if(intervals[i-1].second + CN_EPSILON > interval.first && intervals[i-1].second < interval.second + CN_EPSILON)//c
                                {
                                    intervals[i-1].second = interval.second;
                                    inserted = true;
                                    break;
                                }
                            intervals.insert(intervals.begin() + i, interval);//b
                            inserted = true;
                        }
                    }
                }
                if(intervals.back().second + CN_EPSILON > interval.first && intervals.back().second < interval.second + CN_EPSILON)
                    intervals.back().second = interval.second;
                else if(!inserted)
                    intervals.push_back(interval);
                collision_intervals.at(std::make_pair(con.i1, con.j1)) = intervals;
            }
        }
        else
        {
            Move move(con);
            std::vector<Move> m_cons(0);
            if(constraints.count(move) == 0)
                constraints.insert({move, {move}});
            else
            {
                m_cons = constraints.at(move);
                bool inserted(false);
                for(unsigned int i = 0; i < m_cons.size(); i++)
                {
                    if(inserted)
                        break;
                    if(m_cons[i].t1 > move.t1)
                    {
                        if(m_cons[i].t1 < move.t2 + CN_EPSILON)
                        {
                            m_cons[i].t1 = move.t1;
                            if(move.t2 + CN_EPSILON > m_cons[i].t2)
                                m_cons[i].t2 = move.t2;
                            inserted = true;
                            if(i != 0)
                                if(m_cons[i-1].t2 + CN_EPSILON > move.t1 && m_cons[i-1].t2 < move.t2 + CN_EPSILON)
                                {
                                    m_cons[i-1].t2 = move.t2;
                                    if(m_cons[i-1].t2 + CN_EPSILON > m_cons[i].t1 && m_cons[i-1].t2 < m_cons[i].t2 + CN_EPSILON)
                                    {
                                        m_cons[i-1].t2 = m_cons[i].t2;
                                        m_cons.erase(m_cons.begin() + i);
                                    }
                                    inserted = true;
                                }
                        }
                        else
                        {
                            if(i != 0)
                                if(m_cons[i-1].t2 + CN_EPSILON > move.t1 && m_cons[i-1].t2 < move.t2 + CN_EPSILON)
                                {
                                    m_cons[i-1].t2 = move.t2;
                                    inserted = true;
                                    break;
                                }
                            m_cons.insert(m_cons.begin() + i, move);
                            inserted = true;
                        }
                    }
                }
                if(m_cons.back().t2 + CN_EPSILON > move.t1 && m_cons.back().t2 < move.t2 + CN_EPSILON)
                    m_cons.back().t2 = move.t2;
                else if(!inserted)
                    m_cons.push_back(move);
                constraints.at(move) = m_cons;
            }
        }
    }
}

Path SIPP::find_path(Agent agent, const Map &map, std::list<Constraint> cons, Heuristic* h_values)
{
    this->clear();
    //std::cout<<"find path\n";
    make_constraints(cons);
    //std::cout<<constraints.size()<<"c_size\n";
    if(cons.size() > 100000)
    {
        for(auto con:cons)
            std::cout<<con.agent<<" "<<con.i1<<" "<<con.j1<<" "<<con.i2<<" "<<con.j2<<" "<<con.t1<<" "<<con.t2<<" \n";
        std::cout<<std::endl;
        for(auto it = collision_intervals.begin(); it != collision_intervals.end(); it++)
        {
            std::cout<<it->first.first<<" "<<it->first.second<<"\n";
            for(int i=0; i<it->second.size(); i++)
                std::cout<<it->second[i].first<<" "<<it->second[i].second<<" collsiion interval\n";
        }
    }
    this->h_values = h_values;
    this->agent = agent;
    open.resize(map.height);
    Node curNode(agent.start_i, agent.start_j, 0, 0);
    curNode.g = 0;
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
        //if(agent.id == 5 && curNode.i==25 && curNode.j==31)
        //    std::cout<<"   "<<curNode.g<<" "<<curNode.parent->g<<"   ";
        open[curNode.i].pop_front();
        openSize--;
        close.insert({curNode.i * map.width + curNode.j, curNode});
        if(curNode.i == agent.goal_i && curNode.j == agent.goal_j && curNode.interval.second == CN_INFINITY)
        {
            //std::cout<<agent.id<<" "<<curNode.i<<" "<<curNode.j<<" "<<curNode.g<<" "<<curNode.interval.first<<" "<<curNode.interval.second<<"\n";
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
                add_open(*it);
            it++;
        }
    }
    if (pathFound)
    {
        //std::cout<<"path found\n";
        reconstruct_path(curNode);
        path.cost = curNode.g;
    }
    //std::cout<<agent.id<<" "<<path.cost<<"\n";
    path.agentID = agent.id;
    return path;
}
