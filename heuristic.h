#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "structs.h"
#include "const.h"
#include <vector>
#include <unordered_map>
#include "map.h"
#include <iomanip>

class Heuristic
{
    std::vector<std::vector<std::vector<double>>> h_values;
    std::vector<std::pair<int, int>> moves_2k;
    unsigned int openSize;
    std::vector<std::list<Node>> open;
    Node find_min(int size);
    void add_open(Node newNode);
public:
    Heuristic(){}
    void init(int height, int width, int agents);
    void count(const Map &map, Agent agent);
    int get_value(int i, int j, int id) { return h_values[i][j][id]; }


};

#endif // HEURISTIC_H
