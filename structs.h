#ifndef STRUCTS_H
#define STRUCTS_H
#include <math.h>
#include <vector>
#include <list>
#include <iostream>
#include <chrono>

struct Agent
{
    int start_i, start_j;
    int goal_i, goal_j;
    int id;
    double size;
    Agent(int s_i = -1, int s_j = -1, int g_i = -1, int g_j = -1, int _id = -1)
        :start_i(s_i), start_j(s_j), goal_i(g_i), goal_j(g_j), id(_id) {}
};

struct Node
{
    int     i, j;
    double  f, g;
    Node*   parent;
    std::pair<double, double> interval;

    Node(int _i = -1, int _j = -1, double _f = -1, double _g = -1, Node* _parent = nullptr, double begin = -1, double end = -1)
        :i(_i), j(_j), f(_f), g(_g), parent(_parent), interval(std::make_pair(begin, end)) {}

    ~Node() { parent = nullptr; }
};

struct dNode
{
    double  i, j, g;
    dNode(double _i = -1, double _j = -1, double _g = -1)
        :i(_i), j(_j), g(_g) {}
    dNode(const Node& node): i(node.i), j(node.j), g(node.g) {}

};

struct Path
{
    std::vector<Node> nodes;
    double cost;
    int agentID;
    Path(std::vector<Node> _nodes = std::vector<Node>(0), double _cost = -1, int _agentID = -1)
        : nodes(_nodes), cost(_cost), agentID(_agentID) {}
};

struct Constraint
{
    int agent;
    double t1, t2; // prohibited to start moving from (i1, j1) to (i2, j2) during interval (t1, t2)
    int i1, j1, i2, j2; // in case of node constraint i1==i2, j1==j2.
    Constraint(int _agent = -1, double _t1 = -1, double _t2 = -1, int _i1 = -1, int _j1 = -1, int _i2 = -1, int _j2 = -1)
        : agent(_agent), t1(_t1), t2(_t2), i1(_i1), j1(_j1), i2(_i2), j2(_j2){}
    friend std::ostream& operator <<(std::ostream& os, const Constraint& con)
    {
        os<<con.agent<<" "<<con.t1<<" "<<con.t2<<" "<<con.i1<<" "<<con.j1<<" "<<con.i2<<" "<<con.j2<<"\n";
        return os;
    }
};

struct Move
{
    double t1, t2; // t2 is required for wait action
    int i1, j1, i2, j2; // in case of wait action i1==i2, j1==j2
    Move(double _t1 = -1, double _t2 = -1, int _i1 = -1, int _j1 = -1, int _i2 = -1, int _j2 = -1)
        : t1(_t1), t2(_t2), i1(_i1), j1(_j1), i2(_i2), j2(_j2) {}
    Move(const Move& move) : t1(move.t1), t2(move.t2), i1(move.i1), j1(move.j1), i2(move.i2), j2(move.j2) {}
    Move(const Constraint& con) : t1(con.t1), t2(con.t2), i1(con.i1), j1(con.j1), i2(con.i2), j2(con.j2) {}
    Move(Node a, Node b) : t1(a.g), t2(b.g), i1(a.i), j1(a.j), i2(b.i), j2(b.j) {}
    bool operator <(const Move& other) const
    {
        if     (i1 < other.i1) return true;
        else if(i1 > other.i1) return false;
        else if(j1 < other.j1) return true;
        else if(j1 > other.j1) return false;
        else if(i2 < other.i2) return true;
        else if(i2 > other.i2) return false;
        else if(j2 < other.j2) return true;
        else                   return false;
    }
};

struct Conflict
{
    int agent1, agent2;
    double t;
    Move move1, move2;
    Conflict(int _agent1 = -1, int _agent2 = -1, Move _move1 = Move(), Move _move2 = Move(), double _t = -1)
        : agent1(_agent1), agent2(_agent2), t(_t), move1(_move1), move2(_move2) {}
    Constraint get_constraint(int agent)
    {
        if(agent == 1)
            return Constraint(agent1, t, move1.i1, move1.j1, move1.i2, move1.j2);
        else
            return Constraint(agent2, t, move2.i1, move2.j1, move2.i2, move2.j2);
    }
};

struct CBS_Node
{
    std::vector<Path> paths;
    CBS_Node* parent;
    Constraint constraint;
    double cost;
    int cons_num;
    int conflicts_num;
    CBS_Node(std::vector<Path> _paths = {}, CBS_Node* _parent = nullptr, Constraint _constraint = Constraint(), double _cost = 0, int _cons_num = 0, int _conflicts_num = 0)
        :paths(_paths), parent(_parent), constraint(_constraint), cost(_cost), cons_num(_cons_num), conflicts_num(_conflicts_num) {}
    ~CBS_Node()
    {
        parent = nullptr;
        paths.clear();
    }

};

struct Open_Elem
{
    CBS_Node* tree_pointer;
    double cost;
    int cons_num;
    int conflicts_num;

    Open_Elem(CBS_Node* _tree_pointer = nullptr, double _cost = -1, int _cons_num = -1, int _conflicts_num = -1)
        : tree_pointer(_tree_pointer), cost(_cost), cons_num(_cons_num), conflicts_num(_conflicts_num) {}
    ~Open_Elem()
    {
        tree_pointer = nullptr;
    }
};

class CBS_Tree
{
    std::list<CBS_Node> tree;
    std::list<Open_Elem> open;
public:
    int get_open_size()
    {
        return open.size();
    }

    void add_node(CBS_Node node)
    {
        tree.push_back(node);
        bool inserted = false;
        for(auto it = open.begin(); it != open.end(); it++)
        {
            if(it->cost > node.cost || (it->cost == node.cost && node.cons_num < it->cons_num))
            {
                open.emplace(it, Open_Elem(&tree.back(), node.cost, node.cons_num, node.conflicts_num));
                inserted = true;
                break;
            }
        }
        if(!inserted)
            open.emplace_back(Open_Elem(&tree.back(), node.cost, node.cons_num));
    }

    CBS_Node* get_front()
    {
        return open.begin()->tree_pointer;
    }

    void pop_front()
    {
        open.pop_front();
        return;
    }

    std::vector<Path> get_paths(CBS_Node node, int size)
    {
        std::vector<Path> paths(size);
        while(node.parent != nullptr)
        {
            if(paths.at(node.paths.begin()->agentID).nodes.empty())
                paths.at(node.paths.begin()->agentID) = *node.paths.begin();
            node = *node.parent;
        }
        for(unsigned int i = 0; i < node.paths.size(); i++)
            if(paths.at(i).nodes.empty())
                paths.at(i) = node.paths.at(i);
        return paths;
    }

};

struct Solution
{
    double flowtime;
    double makespan;
    std::chrono::duration<double> time;
    std::vector<Path> paths;
    Solution(double _flowtime = -1, double _makespan = -1, std::vector<Path> _paths = {})
        : flowtime(_flowtime), makespan(_makespan), paths(_paths) {}
    ~Solution() { paths.clear(); }
};

class Vector2D {
  public:
    Vector2D(double _i = 0.0, double _j = 0.0):i(_i),j(_j){}
    double i, j;

    inline Vector2D operator +(const Vector2D &vec) { return Vector2D(i + vec.i, j + vec.j); }
    inline Vector2D operator -(const Vector2D &vec) { return Vector2D(i - vec.i, j - vec.j); }
    inline Vector2D operator -() { return Vector2D(-i,-j); }
    inline Vector2D operator /(const double &num) { return Vector2D(i/num, j/num); }
    inline Vector2D operator *(const double &num) { return Vector2D(i*num, j*num); }
    inline double operator *(const Vector2D &vec){ return i*vec.i + j*vec.j; }
    inline void operator +=(const Vector2D &vec) { i += vec.i; j += vec.j; }
    inline void operator -=(const Vector2D &vec) { i -= vec.i; j -= vec.j; }
};

#endif // STRUCTS_H
