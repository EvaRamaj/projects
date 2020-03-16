
#include <connect4_graph.h>
#include <iostream>
#include <cassert>

//#define TEST_DEBUG
#ifdef TEST_DEBUG
bool compare_states(connect4_state& s1, connect4_state& s2){
    for(int c=0; c<COLUMNS; c++){
        for(int r=0; r<ROWS; r++){
            if(s1.board[c][r] != s2.board[c][r]) return false;
        }
    }
    return true;
}
#endif

using namespace std;


GameGraph::GameGraph(connect4_state& root_state){
    bool is_mirrored = Connect4::isMirrored(root_state);
    root = new graph_node;
    root->state = root_state;
    if(is_mirrored) Connect4::convertMirrored(root->state);
    uint64_t key = Connect4::stateToKey(root_state, is_mirrored);
    root->key = key;
    visited_states.insert(node_pair(key, root));
    leaves.insert(node_pair(key, root));
    depth = 0;
}

GameGraph::~GameGraph(){
    cout << "Deleting graph!" << endl;
    for(auto const& pair : visited_states){
        delete pair.second;
    }
}

void GameGraph::explore_to_depth(int depth_limit){
    cout << depth_limit-depth << " layers to be explored" << endl;
    for(int d = depth; d<depth_limit; d++){
        explore_leaves();
    }
}


bool GameGraph::explore_node(graph_node& node, int c){
    if(!Connect4::canDrop(c, node.state)) return false;

    connect4_state newState = node.state;
    Connect4::dropPiece(c, newState);
    bool is_mirrored = Connect4::isMirrored(newState);
    int c_mirrored = is_mirrored ? COLUMNS-c-1 : c;
    uint64_t key = Connect4::stateToKey(newState, is_mirrored);
    auto it = visited_states.find(key);
    if(it != visited_states.end()){
        //state already exists -> extend pointers
        graph_node *newNode = visited_states[key];
        node.next[c] = newNode;
        newNode->prev[c_mirrored] = &node;

#ifdef TEST_DEBUG           
        assert(newNode->prev[c_mirrored] == nullptr || newNode->prev[c_mirrored]==&node);
        connect4_state& state2 = newNode->state;
        if(is_mirrored) Connect4::convertMirrored(newState);
        if(!compare_states(newState, state2)){
            printState(newState);
            printState(state2);
            cout << c << endl;
            cout << Connect4::stateToKey(newState, false) << endl;
            cout << Connect4::stateToKey(state2, false) << endl;
            cout << visited_states.count(key) << endl;
        }
        assert(compare_states(newState, state2));
#endif
        return false;
    }

    //we only save the non-mirrored states
    if(is_mirrored) Connect4::convertMirrored(newState);

    graph_node *newNode = new graph_node;
    newNode->state = newState;
    newNode->key = key;
    newNode->prev[c_mirrored] = &node;

    int game_state = evaluate_state(newNode);
    if(game_state == STATE_GAME_OVER){
        delete newNode;
        return false;
    }
    visited_states.insert(node_pair(key, newNode));
    //only add non-winnable states as new leaves
    if(game_state == STATE_PLAYABLE) leaves.insert(node_pair(key, newNode));
    else if(game_state == STATE_WINNABLE) leavesWinnable.insert(node_pair(key, newNode));

    node.next[c] = newNode;

    return true;
}


bool GameGraph::explore_leaves(){

    map<uint64_t, graph_node*> oldLeaves;
    leaves.swap(oldLeaves);
    depth++;

    for(auto const& leave_pair : oldLeaves){
        graph_node* node = leave_pair.second;
        bool new_nodes = false;
        for(int c=0; c<COLUMNS; c++){
            if(explore_node(*node, c)){
                new_nodes = true;
            }
        }

        if(new_nodes) propagate_values(node);
    }

    if(leaves.size() == 0){
        cout << "No new leaves explored." << endl;
        if(leavesWinnable.size() > 0){
            leaves.swap(leavesWinnable);
            leavesWinnable.clear(); //should now already be empty
            cout << "Adding " << leaves.size() << " winnable leaves instead." << endl;
        }
    }else{
        cout << leaves.size() << " new nodes explored" << endl;
    }
    return leaves.size() > 0;
}

bool GameGraph::prune_move(int column){
    if(root->next[column] == nullptr){
        explore_node(*root, column);
        if(root->next[column] == nullptr){
            return false;
        }
    }
    graph_node *oldRoot = root;
    root = root->next[column];
    if(oldRoot->next[column] == oldRoot->next[COLUMNS-column-1])
        oldRoot->next[COLUMNS-column-1] = nullptr;
    oldRoot->next[column] = nullptr;
    for(int c=0; c<COLUMNS; c++) root->prev[c] = nullptr;

    int num_nodes = visited_states.size();
    int num_pruned = prune_tree(oldRoot, (graph_node *)nullptr);
    cout << "Pruned " << num_pruned << " leaves, and " << (num_nodes-visited_states.size()) << " nodes overall" << endl;
    cout << "Nodes left: " << visited_states.size() << endl;
    depth--;
    return true;
}


int GameGraph::prune_tree(graph_node *tree, graph_node *prevTree){
    //1. Fix parents, check if there's still a parent
    bool all_null = true;
    if(prevTree != nullptr){
        //if(tree->prev[fromColumn] == tree->prev[COLUMNS-fromColumn-1])
        //    tree->prev[COLUMNS-fromColumn-1] = nullptr;
        //tree->prev[fromColumn] = nullptr;
        for(int c=0; c<COLUMNS; c++){
            if(tree->prev[c] == prevTree) tree->prev[c] = nullptr;
            else if(tree->prev[c] != nullptr) all_null = false;
        }
        if(!all_null) return 0;
    }
    //2. If there is no parent, delete this node and propagate this to children
    visited_states.erase(tree->key);

    int num_pruned = 0;
    all_null = true;
    for(int c=0; c<COLUMNS; c++){
        //mirror node that was already explored
        if(c>COLUMNS/2 && tree->next[c] == tree->next[COLUMNS-c-1]) continue;
        if(tree->next[c] != nullptr){
            all_null = false;
#ifdef TEST_DEBUG
            for(int c2=0; c2<c; c2++) assert(tree->next[c] != tree->next[c2]);
            assert(tree->next[c] != tree);
            assert(!compare_states(tree->state, tree->next[c]->state));
#endif
            num_pruned += prune_tree(tree->next[c], tree);
        }
    }

    //3. If this is a leave node, mark it for removal
    if(all_null){
        leaves.erase(tree->key);
        leavesWinnable.erase(tree->key);
        num_pruned++;
    }

    delete tree;
    return num_pruned;
}


void GameGraph::propagate_values(graph_node* node){
    bool updated = false;
    for(int c=0; c<COLUMNS; c++){
        if(node->next[c] != nullptr){
            //negative because players switched
            float value = -node->next[c]->evaluation.value; 
            if(node->evaluation.value < value){
                node->evaluation.value = value;
                updated = true;
            }
        }
    }

    if(node == root || !updated) return;

    for(int c=0; c<COLUMNS; c++){
        if(node->prev[c] != nullptr){
            propagate_values(node->prev[c]);
        }
    }
}



int GameGraph::evaluate_state(graph_node* node){
    int movesRed, movesBlue;
    int game_state = Connect4::countWinningMoves(node->state, movesRed, movesBlue);
    int movesPlayer = node->state.nextPlayer == RED ? movesRed : movesBlue;
    int movesOther = node->state.nextPlayer == RED ? movesBlue : movesRed;
    float value = 0;

    if(game_state == GAME_ALREADY_OVER) return STATE_GAME_OVER;
    game_state = STATE_PLAYABLE;

    if(movesPlayer > 0){
        game_state = STATE_WINNABLE;
        value = (float)movesPlayer;
    }else if(movesOther == 1){
        //might be favorable situation actually
        //value = -0.5;
    }else if(movesOther >= 2){
        game_state = STATE_WINNABLE;
        value = -1.;
    }

    node->evaluation.value = value;
    return game_state;
}


void GameGraph::get_root_evaluation(float (&evaluation)[COLUMNS]){
    for(int c=0; c<COLUMNS; c++){
        if(root->next[c] == nullptr){
            evaluation[c] = 0;
        }else{
            evaluation[c] = -root->next[c]->evaluation.value;
        }
    } 
}

void GameGraph::print_graph(){
    cout << "----- GameGraph" << endl;
    cout << "Root node: " << root->key << endl;
    for(auto const& pair : visited_states){
        graph_node * node = pair.second;
        cout << "Node " << node->key << ":" << endl;
        cout << "Next: {";
        for(int c=0; c<COLUMNS; c++){
            if(node->next[c] == nullptr) cout << " NULL";
            else cout << " " << node->next[c]->key;
            if(c!=COLUMNS-1) cout << ",";
        }
        cout << " }" << endl;
        cout << "Prev: {";
        for(int c=0; c<COLUMNS; c++){
            if(node->prev[c] == nullptr) cout << " NULL";
            else cout << " " << node->prev[c]->key;
            if(c!=COLUMNS-1) cout << ",";
        }
        cout << " }" << endl;
        printState(node->state);
    }
    cout << "------------------------------" << endl;
}

