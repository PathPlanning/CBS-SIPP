#ifndef MOVES_H
#define MOVES_H
#include <vector>
#include "map.h"
#include "structs.h"
struct Step
{
    int i;
    int j;
    double cost;
    Step(const Node& node): i(node.i), j(node.j), cost(node.g) {}
    Step(int _i = 0, int _j = 0): i(_i), j(_j), cost(-1.0) {}
};

class Moves
{
private:
    std::vector<Step> moves;
public:
    Moves(){}
    void generate(int k)
    {
        moves.clear();
        if(k == 2)
            moves = {{0,1}, {1,0}, {-1,0},  {0,-1}};
        else if(k == 3)
            moves = {{0,1}, {1,1}, {1,0},  {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1}};
        else if(k == 4)
            moves = {{0,1}, {1,1}, {1,0},  {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1},
                     {1,2}, {2,1}, {2,-1}, {1,-2}, {-1,-2}, {-2,-1}, {-2,1},  {-1,2}};
        else
            moves = {{0,1},   {1,1},   {1,0},   {1,-1},  {0,-1},  {-1,-1}, {-1,0}, {-1,1},
                     {1,2},   {2,1},   {2,-1},  {1,-2},  {-1,-2}, {-2,-1}, {-2,1}, {-1,2},
                     {1,3},   {2,3},   {3,2},   {3,1},   {3,-1},  {3,-2},  {2,-3}, {1,-3},
                     {-1,-3}, {-2,-3}, {-3,-2}, {-3,-1}, {-3,1},  {-3,2},  {-2,3}, {-1,3}};
        for(int i = 0; i < moves.size(); i++)
            moves[i].cost = sqrt(pow(moves[i].i, 2) + pow(moves[i].j, 2));
    }

    std::vector<Step> get_valid(int i, int j, const Map& map)
    {
        std::vector<bool> valid(moves.size(), true);
        if(CN_K == 2)
        {
            for(int k = 0; k < 4; k++)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    valid[k] = false;
        }
        else if(CN_K == 3)
        {
            for(int k = 0; k < 8; k += 2)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                {
                    valid[k-1 < 0 ? 7 : k - 1] = false; valid[k] = false; valid[k + 1] = false;
                }
            for(int k = 1; k < 8; k += 2)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    valid[k] = false;
        }
        else if(CN_K == 4)
        {
            for(int k = 0; k < 8; k += 2)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                {
                    if(k == 0)
                    {
                        valid[7]  = false;
                        valid[15] = false;
                    }
                    else
                    {
                        valid[k - 1] = false;
                        valid[k + 7] = false;
                    }
                    valid[k]     = false;
                    valid[k + 1] = false;
                    valid[k + 8] = false;
                }
            for(int k = 1; k < 8; k += 2)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                {
                    valid[k]     = false;
                    valid[k + 7] = false;
                    valid[k + 8] = false;
                }
            for(int k = 8; k < 16; k++)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    valid[k] = false;
        }
        else
        {
            for(int k = 0; k < 8; k += 2)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                {
                    if(k == 0)
                    {
                        valid[7]  = false;
                        valid[15] = false;
                        valid[30] = false;
                        valid[31] = false;
                    }
                    else
                    {
                        valid[k - 1]    = false;
                        valid[k + 7]    = false;
                        valid[k*2 + 14] = false;
                        valid[k*2 + 15] = false;
                    }
                    valid[k]        = false;
                    valid[k + 1]    = false;
                    valid[k + 8]    = false;
                    valid[k*2 + 16] = false;
                    valid[k*2 + 17] = false;
                }
            for(int k = 1; k < 8; k += 2)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                {
                    valid[k]     = false;
                    valid[k + 7] = false;
                    valid[k + 8] = false;
                    for(int l = 14; l < 18; l++)
                        valid[k*2 + l] = false;
                }
            for(int k = 8; k < 16; k++)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                {
                    valid[k]       = false;
                    valid[k*2]     = false;
                    valid[k*2 + 1] = false;
                }
            for(int k = 16; k < 32; k++)
                if(!map.cell_on_grid(i + moves[k].i, j + moves[k].j) || map.cell_is_obstacle(i + moves[k].i, j + moves[k].j))
                    valid[k]       = false;
        }

        std::vector<Step> valid_moves = {};
        for(int k = 0; k < valid.size(); k++)
            if(valid[k])
                valid_moves.push_back(moves[k]);
        return valid_moves;
    }
};

#endif // MOVES_H
