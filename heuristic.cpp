#include "heuristic.h"

void Heuristic::init(int height, int width, int agents)
{
    h_values.resize(height);
    for(int i = 0; i < height; i++)
    {
        h_values[i].resize(width);
        for(int j = 0; j < width; j++)
            h_values[i][j].resize(agents, -1);
    }
}

void Heuristic::count(const Map& map, Agent agent)
{
    open.clear();
    open.resize(map.get_height());
    openSize = 0;
    Node curNode(agent.goal_i, agent.goal_j, 0, 0), newNode;
    add_open(curNode);
    while(openSize > 0)
    {
        do curNode = find_min(map.get_height());
        while(h_values[curNode.i][curNode.j][agent.id] >= 0 && openSize > 0);
        if(h_values[curNode.i][curNode.j][agent.id] < 0)
            h_values[curNode.i][curNode.j][agent.id] = curNode.g;
        std::vector<Step> valid_moves = map.get_valid_moves(curNode.i, curNode.j);
        for(auto move: valid_moves)
        {
            newNode.i = curNode.i + move.i;
            newNode.j = curNode.j + move.j;
            newNode.g = curNode.g + 1;
            if(h_values[newNode.i][newNode.j][agent.id] < 0)
                add_open(newNode);
        }
    }
    /*for(int i=0; i<map.get_height(); i++)
    {
        for(int j=0; j<map.get_width(); j++)
            std::cout<<std::setw(8)<<h_values[i][j][agent.id]<<" ";
        std::cout<<std::endl;
    }
    std::cout<<std::endl<<std::endl;*/
}

void Heuristic::add_open(Node newNode)
{
    for(auto iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
    {
        if (iter->g > newNode.g)
        {
            openSize++;
            open[newNode.i].insert(iter, newNode);
            return;
        }
        if (iter->j == newNode.j)
            return;
    }
    openSize++;
    open[newNode.i].push_back(newNode);
    return;
}

Node Heuristic::find_min(int size)
{
    Node min;
    min.g = CN_INFINITY;
    for(int i = 0; i < size; i++)
        if(!open[i].empty())
            if(open[i].begin()->g < min.g)
                min = *open[i].begin();
    open[min.i].pop_front();
    openSize--;
    return min;
}

