#include "heuristic.h"

void Heuristic::init(int height, int width, int agents, int k)
{
    h_values.resize(height);
    for(int i = 0; i < height; i++)
    {
        h_values[i].resize(width);
        for(int j = 0; j < width; j++)
            h_values[i][j].resize(agents, -1);
    }
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

void Heuristic::count(const Map& map, Agent agent)
{
    open.clear();
    open.resize(map.height);
    openSize = 0;
    Node curNode(agent.goal_i, agent.goal_j, 0, 0), newNode;
    add_open(curNode);
    while(openSize > 0)
    {
        do curNode = find_min(map.height);
        while(h_values[curNode.i][curNode.j][agent.id] >= 0 && openSize > 0);
        if(h_values[curNode.i][curNode.j][agent.id] < 0)
            h_values[curNode.i][curNode.j][agent.id] = curNode.g;
        for(auto move: moves_2k)
        {
            newNode.i = curNode.i + move.first;
            newNode.j = curNode.j + move.second;
            if(map.cell_on_grid(newNode.i, newNode.j) && map.cell_is_traversable(newNode.i, newNode.j)
                    && los.checkLine(newNode.i, newNode.j, curNode.i, curNode.j, map, CN_AGENT_SIZE))
            {
                newNode.g = curNode.g + dist(newNode, curNode);
                if(h_values[newNode.i][newNode.j][agent.id] < 0)
                    add_open(newNode);
            }
        }
    }
}

void Heuristic::add_open(Node newNode)
{
    std::list<Node>::iterator iter;
    for(iter = open[newNode.i].begin(); iter != open[newNode.i].end(); ++iter)
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
    open[newNode.i].insert(iter, newNode);
    return;
}

Node Heuristic::find_min(int size)
{
    Node min;
    for(int i = 0; i < size; i++)
        if(!open[i].empty())
            if(open[i].begin()->g < min.g || min.g == -1)
                min = *open[i].begin();
    open[min.i].pop_front();
    openSize--;
    return min;
}

