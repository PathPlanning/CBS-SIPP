#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H
#include "map.h"

struct LineOfSight
{
    bool checkLine(int x1, int y1, int x2, int y2, const Map &map, double agentSize)
    {
        int delta_x(std::abs(x1 - x2));
        int delta_y(std::abs(y1 - y2));
        if((delta_x > delta_y && x1 > x2) || (delta_y >= delta_x && y1 > y2))
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int step_x(x1 < x2 ? 1 : -1);
        int step_y(y1 < y2 ? 1 : -1);
        int error(0), x(x1), y(y1);
        int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON;
        int k, num;

        if(delta_x > delta_y)
        {
            int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_y;
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    if(map.cell_is_obstacle(x1 - n*step_x, y1 + k*step_y))
                        return false;
                for(k = 1; k <= num; k++)
                    if(map.cell_is_obstacle(x2 + n*step_x, y2 - k*step_y))
                        return false;
            }
            error = 0;
            for(x = x1; x != x2 + step_x; x++)
            {
                if(map.cell_is_obstacle(x, y))
                    return false;
                if(x < x2 - extraCheck)
                {
                    num = (gap + error)/delta_x;
                    for(k = 1; k <= num; k++)
                        if(map.cell_is_obstacle(x, y + k*step_y))
                            return false;
                }
                if(x > x1 + extraCheck)
                {
                    num = (gap - error)/delta_x;
                    for(k = 1; k <= num; k++)
                        if(map.cell_is_obstacle(x, y - k*step_y))
                            return false;
                }
                error += delta_y;
                if((error<<1) > delta_x)
                {
                    y += step_y;
                    error -= delta_x;
                }
            }
        }
        else
        {
            int extraCheck = agentSize*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_x;
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    if(map.cell_is_obstacle(x1 + k*step_x, y1 - n*step_y))
                        return false;
                for(k = 1; k <= num; k++)
                    if(map.cell_is_obstacle(x2 - k*step_x, y2 + n*step_y))
                        return false;
            }
            error = 0;
            for(y = y1; y != y2 + step_y; y += step_y)
            {
                if(map.cell_is_obstacle(x, y))
                    return false;
                if(y < y2 - extraCheck)
                {
                    num = (gap + error)/delta_y;
                    for(k = 1; k <= num; k++)
                        if(map.cell_is_obstacle(x + k*step_x, y))
                            return false;
                }
                if(y > y1 + extraCheck)
                {
                    num = (gap - error)/delta_y;
                    for(k = 1; k <= num; k++)
                        if(map.cell_is_obstacle(x - k*step_x, y))
                            return false;
                }
                error += delta_x;
                if((error<<1) > delta_y)
                {
                    x += step_x;
                    error -= delta_y;
                }
            }
        }
        return true;
    }
    std::vector<std::pair<int, int>> getCellsCrossedByLine(int x1, int y1, int x2, int y2, double agentSize)
    {
        std::vector<std::pair<int, int>> lineCells(0);
        int delta_x = std::abs(x1 - x2);
        int delta_y = std::abs(y1 - y2);
        if((delta_x > delta_y && x1 > x2) || (delta_y >= delta_x && y1 > y2))
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }
        int step_x = (x1 < x2 ? 1 : -1);
        int step_y = (y1 < y2 ? 1 : -1);
        int error = 0, x = x1, y = y1;
        int k, num;
        std::pair<int, int> add;
        int gap = agentSize*sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + double(delta_x + delta_y)/2 - CN_EPSILON;

        if(delta_x > delta_y)
        {
            int extraCheck = agentSize*delta_y/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_y;
                num = (gap - error)/delta_x;
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x1 - n*step_x, y1 + k*step_y});
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x2 + n*step_x, y2 - k*step_y});
            }
            error = 0;
            for(x = x1; x != x2 + step_x; x++)
            {
                lineCells.push_back({x, y});
                if(x < x2 - extraCheck)
                {
                    num = (gap + error)/delta_x;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x, y + k*step_y});
                }
                if(x > x1 + extraCheck)
                {
                    num = (gap - error)/delta_x;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x, y - k*step_y});
                }
                error += delta_y;
                if((error<<1) > delta_x)
                {
                    y += step_y;
                    error -= delta_x;
                }
            }
        }
        else
        {
            int extraCheck = agentSize*delta_x/sqrt(pow(delta_x, 2) + pow(delta_y, 2)) + 0.5 - CN_EPSILON;
            for(int n = 1; n <= extraCheck; n++)
            {
                error += delta_x;
                num = (gap - error)/delta_y;
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x1 + n*step_x, y1 - k*step_y});
                for(k = 1; k <= num; k++)
                    lineCells.push_back({x2 - n*step_x, y2 + k*step_y});
            }
            error = 0;
            for(y = y1; y != y2 + step_y; y += step_y)
            {
                lineCells.push_back({x, y});
                if(y < y2 - extraCheck)
                {
                    num = (gap + error)/delta_y;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x + k*step_x, y});
                }
                if(y > y1 + extraCheck)
                {
                    num = (gap - error)/delta_y;
                    for(k = 1; k <= num; k++)
                        lineCells.push_back({x - k*step_x, y});
                }
                error += delta_x;
                if((error<<1) > delta_y)
                {
                    x += step_x;
                    error -= delta_y;
                }
            }
        }
        /*for(k = 0; k < cells.size(); k++)
        {
            add = {x1 + cells[k].first, y1 + cells[k].second};
            if(std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                lineCells.push_back(add);
            add = {x2 + cells[k].first, y2 + cells[k].second};
            if(std::find(lineCells.begin(), lineCells.end(), add) == lineCells.end())
                lineCells.push_back(add);
        }*/
        return lineCells;
    }
};

#endif // LINEOFSIGHT_H
