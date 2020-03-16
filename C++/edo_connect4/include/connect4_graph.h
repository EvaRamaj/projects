
#ifndef PROJECT_EDOCONNECTFOURGRAPH_H
#define PROJECT_EDOCONNECTFOURGRAPH_H

#include <connect4.h>
#include <map>
#include <set>
#include <vector>
#include <iostream>

#include <stdlib.h>
#include <time.h>

#include <thread>
#include <mutex>
#include <atomic>

#define MAX_LEAVES 10000000 //upper limit for monte carlo search
#define MAX_EXPLORE_LEAVES 50000 //upper limit for explore_to_depth in monte carlo search

#define STATE_PLAYABLE 0
#define STATE_WINNABLE 1
#define STATE_GAME_OVER 2



typedef struct graph_node{
    connect4_state state;
    uint64_t key;
    graph_node *next[COLUMNS] = {nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr};
    graph_node *prev[COLUMNS] = {nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr};
    union{
        struct { int countWon, countTotal; } monte_carlo_values;
        float value;
    } evaluation = {.monte_carlo_values={0,0}};
} graph_node;

typedef std::pair<uint64_t, graph_node*> node_pair;

class GameGraph{
public:
    GameGraph(connect4_state& root_state);
    ~GameGraph();

    //removes all subtrees that don't fit with the recorded move
    //returns true if successfull, false if this subtree hasn't been explored yet
    virtual bool prune_move(int column);

    virtual void get_root_evaluation(float (&evaluation)[COLUMNS]);

    virtual void print_graph();

    //calls explore_leaves until depth is reached
    virtual void explore_to_depth(int depth_limit);

protected:
    graph_node *root;
    std::map<uint64_t, graph_node*> visited_states;
    std::map<uint64_t, graph_node*> leaves;
    std::map<uint64_t, graph_node*> leavesWinnable; //leaves where a player can win
    int depth;

    //explores column of given node, returns true if a new node was created
    virtual bool explore_node(graph_node& node, int column);
    //explores all current leaves
    //returns false if all leaves have been explored
    virtual bool explore_leaves();

    //prunes the given tree and cleans up the lists above along the way
    //returns number of leaves pruned
    virtual int prune_tree(graph_node *tree, graph_node *prevTree);

    //goes back from given leave and fixes all values along the way
    virtual void propagate_values(graph_node* node);
    //fills values of given node, and returns how the node should be handled (see defines above)
    virtual int evaluate_state(graph_node* node);
};

class MonteCarloGraph : public GameGraph {
public:
    MonteCarloGraph(connect4_state& root_state) : GameGraph(root_state) { srand(time(NULL)); }
    ~MonteCarloGraph();

    bool monte_carlo_step(int num_playouts); //execute a monte carlo step on a random leave, returns false if there are no leaves
    void get_root_evaluation(float (&evaluation)[COLUMNS]); //overwritten to fit monte carlo

    void explore_to_depth(int depth_limit); //overwritten for thread safety
    bool prune_move(int column); //overwritten for thread safety

    void start_thread(); //start a new thread
    void stop_all_threads(); //stop all running threads


    void serialize(std::ostream& ss); //serialize using custom format
    void deserialize(std::istream& ss);
    void save_graph(std::string filename="mcts.dat"); //writes graph to file using custom format
    void load_graph(std::string filename="mcts.dat"); //loads graph from custom format

protected:
    bool running = false;
    std::mutex updateLock; //lock when updating/propagating values
    std::mutex exploreLock; //lock when exploring a single leave
    std::set<graph_node *> leavesLocked; //leaves that are being explored
    std::vector<std::thread> threads; //threads running

    void monte_carlo_thread(); //function used by the threads
    void monte_carlo_playout(graph_node* node); //playout a whole game and update values of the node
    void propagate_values(graph_node* node); //overwritten to fit monte carlo search
    void propagate_values(graph_node* node, int countWon, int countTotal); //like propagate_values, with extra parameter
    int evaluate_state(graph_node* node); //overwritten to fit monte carlo search
    bool explore_leaves(); //overwritten to add explore limit

    //no shared mutex/lock in C++11, hence we implement our own here
    std::mutex graphLock, graphCounterLock;
    bool graphLockNeeded = false;
    int graphInUse = 0;
    void lock_graph();
    void unlock_graph();
    void lock_graph_shared();
    void unlock_graph_shared();


    void to_dot(std::ostream& ss); //writes the graph in dot format to the given stream
    void save_dot(); //saves the graph in dot format, used as a helper function for debugging currently
};



#endif //PROJECT_EDOCONNECTFOURGRAPH_H