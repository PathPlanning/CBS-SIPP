#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <vector>
#include "tinyxml2.h"
#include "const.h"

class Map
{
public:
    std::vector<std::vector<int>> grid;
    int height, width;

    Map(){}
    ~Map(){}
    bool get_map(const char* FileName);
    bool cell_is_traversable (int i, int j) const;
    bool cell_on_grid (int i, int j) const;
    bool cell_is_obstacle(int i, int j) const;
    int  get_value(int i, int j) const;
    void print_map();

};

#endif // MAP_H
