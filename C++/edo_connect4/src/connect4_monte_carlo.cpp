
#include <connect4_graph.h>
#include <iostream>
#include <cassert>
#include <fstream>
#include <sstream>
#include <unistd.h>

//#define TEST_DEBUG

using namespace std;

MonteCarloGraph::~MonteCarloGraph(){
	stop_all_threads();
}

bool MonteCarloGraph::monte_carlo_step(int num_playouts){
	//1. Pick random unlocked leave and explore it
    exploreLock.lock();
    if(leaves.size() == 0){
    	cout << "All leaves explored." << endl;
    	if(leavesWinnable.size() > 0){
    		leaves.swap(leavesWinnable);
    		leavesWinnable.clear(); //should now already be empty
            cout << "Adding " << leaves.size() << " winnable leaves instead." << endl;
    	}else{
	    	exploreLock.unlock();
	        return false;
        }
    }else if(leaves.size() <= leavesLocked.size()){
    	cout << "No unlocked leave available." << endl;
    	exploreLock.unlock();
    	usleep(2000000);
    	return true;
    }else if(leaves.size() >= MAX_LEAVES){
    	cout << "Max leaves reached." << endl;
    	exploreLock.unlock();
    	usleep(2000000);
        return true;
    }

    graph_node* leave_to_explore;
    do{
	    auto it = leaves.begin();
	    advance(it, rand()%leaves.size());
	    leave_to_explore = (*it).second;
	}while(leavesLocked.find(leave_to_explore) != leavesLocked.end());
	leaves.erase(leave_to_explore->key);


    std::set<graph_node *> leavesToPlayout;
    for(int c=0; c<COLUMNS; c++){
        if(explore_node(*leave_to_explore, c)){
            graph_node* node = leave_to_explore->next[c];
            //if present, the leave is already being explored by some other thread
            if(leavesLocked.find(node) == leavesLocked.end()){
	            leavesLocked.insert(node);
	            leavesToPlayout.insert(node);
            }
        }
    }
    exploreLock.unlock();

    if(leavesToPlayout.size() == 0) return true;

    //2. Simulate playouts for the newly explored leaves
    int sumWon = 0;
    int sumTotal = 0;
    for(auto node : leavesToPlayout){
        for(int k=0; k<num_playouts; k++){
            monte_carlo_playout(node);
        }
	    sumWon += node->evaluation.monte_carlo_values.countWon;
	    sumTotal += node->evaluation.monte_carlo_values.countTotal;
    }

    //3. Aggregate results, and then propagate these back up
    //update_values calls propagate
	updateLock.lock();
    propagate_values(leave_to_explore, sumTotal-sumWon, sumTotal);
	updateLock.unlock();

    //4. Release locked leaves
    exploreLock.lock();
    for(auto node : leavesToPlayout){
    	leavesLocked.erase(node);
    }
    exploreLock.unlock();
    return true;
}

void MonteCarloGraph::monte_carlo_playout(graph_node* node){
    Connect4 gameRandom(RANDOM);
    gameRandom.setCurrentState(node->state);
    int winner = gameRandom.playGame();
    node->evaluation.monte_carlo_values.countTotal++;
    switch(winner){
    case RED_WON:
        if(node->state.nextPlayer == RED) node->evaluation.monte_carlo_values.countWon++;
        return;
    case BLUE_WON:
        if(node->state.nextPlayer == BLUE) node->evaluation.monte_carlo_values.countWon++;
        return;
    case TIE:
        if(rand()%2) node->evaluation.monte_carlo_values.countWon++;
    	return;
    default:
        return;
    }
}


void MonteCarloGraph::get_root_evaluation(float (&evaluation)[COLUMNS]){
	updateLock.lock();

    for(int c=0; c<COLUMNS; c++){
        if(root->next[c] == nullptr){
            evaluation[c] = 0;
        }else if(root->next[c]->evaluation.monte_carlo_values.countTotal == 0){
        	evaluation[c] = 0.5;
        }else{
            //try to look one more move ahead, if possible
            graph_node* node = root->next[c];
            bool found_child = false;
            evaluation[c] = 1.0;
            for(int c2=0; c2<COLUMNS; c2++){
                if(node->next[c2] != nullptr){
                    found_child = true;
                    double won = node->next[c2]->evaluation.monte_carlo_values.countWon;
                    double total = node->next[c2]->evaluation.monte_carlo_values.countTotal;
                    double win_rate = won/total;
                    //opponent will choose the worst win rate for us
                    if(win_rate < evaluation[c])
                        evaluation[c] = (float)win_rate;
                }
            }

            if(!found_child){
                double won = root->next[c]->evaluation.monte_carlo_values.countWon;
                double total = root->next[c]->evaluation.monte_carlo_values.countTotal;
                evaluation[c] = (float)(1.0 - won/total);
            }
        }
    }

	updateLock.unlock();
}


void MonteCarloGraph::propagate_values(graph_node* node){
	int sumWon = 0;
	int sumTotal = 0;
    for(int c=0; c<COLUMNS; c++){
    	//ignore mirror state
    	if(c > COLUMNS/2 && node->next[c] == node->next[COLUMNS-c-1]) continue;
    	if(node->next[c] != nullptr){
    		sumWon += node->next[c]->evaluation.monte_carlo_values.countWon;
    		sumTotal += node->next[c]->evaluation.monte_carlo_values.countTotal;
    	}
    }
    propagate_values(node, sumTotal-sumWon, sumTotal);
}


void MonteCarloGraph::propagate_values(graph_node* node, int countWon, int countTotal){
	if(countTotal == 0) return;

    set<graph_node*> frontier, frontierNext;
    frontier.insert(node);

    while(frontier.size() > 0){
        for(auto node : frontier){
        	node->evaluation.monte_carlo_values.countWon += countWon;
        	node->evaluation.monte_carlo_values.countTotal += countTotal;
		    if(node->evaluation.monte_carlo_values.countTotal > 1000000){
		    	node->evaluation.monte_carlo_values.countWon /= 1000;
		    	node->evaluation.monte_carlo_values.countTotal /= 1000;
		    }

            for(int c=0; c<COLUMNS; c++){
                if(node->prev[c] != nullptr)
                    frontierNext.insert(node->prev[c]);
            }
        }

        frontier.swap(frontierNext);
        frontierNext.clear();
        countWon = countTotal-countWon;
    }
}



int MonteCarloGraph::evaluate_state(graph_node* node){
    connect4_move moveRed, moveBlue;
    int winner = Connect4::canWin(node->state, moveRed, moveBlue);
    node->evaluation.monte_carlo_values.countWon = 0;
    node->evaluation.monte_carlo_values.countTotal = 1;

    switch(winner){
    case NO_WINNER_YET:
    	node->evaluation.monte_carlo_values.countTotal = 0;
        return STATE_PLAYABLE;
    case TIE:
    	//TIE -> both can win, hence always +1 here
        node->evaluation.monte_carlo_values.countWon = 1;
        return STATE_WINNABLE;
    case RED_WON:
        if(node->state.nextPlayer == RED){
            node->evaluation.monte_carlo_values.countWon = 1;
            return STATE_WINNABLE;
        }
        return STATE_PLAYABLE;
    case BLUE_WON:
        if(node->state.nextPlayer == BLUE){
            node->evaluation.monte_carlo_values.countWon = 1;
            return STATE_WINNABLE;
        }
        return STATE_PLAYABLE;
    case GAME_ALREADY_OVER:
    	return STATE_GAME_OVER;
    }
    assert(false);
}

void MonteCarloGraph::monte_carlo_thread(){
	bool leaves_available = true;
	while(running && leaves_available){
		lock_graph_shared();
		leaves_available = monte_carlo_step(50);
		unlock_graph_shared();
	}
	cout << "Thread exiting." << endl;
}

void MonteCarloGraph::start_thread(){
	running = true;
	thread t(&MonteCarloGraph::monte_carlo_thread, this);
	threads.push_back(move(t));
	cout << "Started thread!" << endl;
}

void MonteCarloGraph::stop_all_threads(){
	running = false;
	for(int i=0; i<threads.size(); i++)
		threads[i].join();
}

bool MonteCarloGraph::explore_leaves(){
	if(leaves.size() >= MAX_EXPLORE_LEAVES) return true;
	return GameGraph::explore_leaves();
}

void MonteCarloGraph::explore_to_depth(int depth_limit){
	lock_graph();
	GameGraph::explore_to_depth(depth_limit);
	unlock_graph();
}

bool MonteCarloGraph::prune_move(int column){
	lock_graph();
	bool states_left = GameGraph::prune_move(column);
	if(!states_left) running = false;
	unlock_graph();
	return states_left;
}



void MonteCarloGraph::lock_graph(){
	graphLockNeeded = true;
	graphLock.lock();
	graphLockNeeded = false;
}
void MonteCarloGraph::unlock_graph(){
	graphLock.unlock();
}
void MonteCarloGraph::lock_graph_shared(){
	while(graphLockNeeded); //wait until priority thread acquires lock -> makes it writer-preferring
	graphCounterLock.lock();
	graphInUse++;
	if(graphInUse == 1) graphLock.lock();
	graphCounterLock.unlock();
}
void MonteCarloGraph::unlock_graph_shared(){
	graphCounterLock.lock();
	graphInUse--;
	if(graphInUse == 0) graphLock.unlock();
	graphCounterLock.unlock();
}


void to_dot_helper(ostream& ss, graph_node* node, std::map<uint64_t, graph_node*>& plotted_states){
	if(plotted_states.count(node->key) > 0) return;
	plotted_states.insert(node_pair(node->key, node));

	ss << node->key << " [label=\"";
	ss << node->key << ":";
	ss << node->evaluation.monte_carlo_values.countWon << "/";
	ss << node->evaluation.monte_carlo_values.countTotal << "\"];" << endl;

	vector<graph_node*> nextNodes;
	for(int c=0; c<COLUMNS; c++){
		if(node->next[c] != nullptr){
			ss << node->key << " -> " << node->next[c]->key << ";" << endl;;
			nextNodes.push_back(node->next[c]);
		}
	}

	for(auto node : nextNodes){
		to_dot_helper(ss, node, plotted_states);
	}
}


void MonteCarloGraph::to_dot(ostream& ss){
	lock_graph();
	ss << "digraph MonteCarloTree { " << endl;
	std::map<uint64_t, graph_node*> plotted_states;
	to_dot_helper(ss, root, plotted_states);
	ss << "}" << endl;
	unlock_graph();
}


int fileCounter = 0;
void MonteCarloGraph::save_dot(){
	ofstream graphFile;
	stringstream filename;
    filename << "graph_mc_" << fileCounter++ << ".gv";
	graphFile.open(filename.str());
	cout << "Creating dot graph." << endl;
	to_dot(graphFile);
	graphFile.close();
	cout << "Dot graph saved." << endl;
}




void MonteCarloGraph::serialize(ostream& ss){
	lock_graph();
    ss << visited_states.size() << " " << root->key << endl;
    for(auto const& pair : visited_states){
        graph_node * node = pair.second;
        ss << node->key << " ";

        //WRITE NEXT/PREV
        for(int c=0; c<COLUMNS; c++){
            if(node->next[c] == nullptr) ss << node->key;
            else ss << node->next[c]->key;
            ss << " ";
        }
        for(int c=0; c<COLUMNS; c++){
            if(node->prev[c] == nullptr) ss << node->key;
            else ss << node->prev[c]->key;
            ss << " ";
        }

        //WRITE MONTE CARLO VALUES
        ss << node->evaluation.monte_carlo_values.countWon << " ";
        ss << node->evaluation.monte_carlo_values.countTotal << " ";

        //WRITE CONNECT4 STATE
        ss << node->state.nextPlayer << " ";
        for(int c=0; c<COLUMNS; c++){
        	for(int r=0; r<ROWS; r++){
        		ss << (int)node->state.board[c][r] << " ";
        	}
        }
        ss << endl;
    }
	unlock_graph();
}

void MonteCarloGraph::deserialize(istream& ss){
	lock_graph();
	size_t num_nodes;
	uint64_t root_key;
	ss >> num_nodes;
	ss >> root_key;

	if(root_key != root->key){
		throw "Different root states!";
	}

	for(size_t i=0; i<num_nodes; i++){
		graph_node * node;
		uint64_t key;

		//READ KEY
		ss >> key;
    	auto it = visited_states.find(key);
    	if(it == visited_states.end()){
    		node = new graph_node;
    		visited_states.insert(node_pair(key, node));
    	}else node = (*it).second;
    	node->key = key;

    	// READ NEXT/PREV
    	bool found_next = false;
        for(int c=0; c<COLUMNS*2; c++){
        	ss >> key;
	        graph_node * node_for_key = nullptr;
        	if(key != node->key){
        		//if we find a valid next pointer, then this isn't a leave node
        		if(c < COLUMNS) found_next = true;

	        	auto it = visited_states.find(key);
	        	if(it == visited_states.end()){
	        		//node doesn't exist yet
	        		node_for_key = new graph_node;
	        		visited_states.insert(node_pair(key, node_for_key));
	        	}else node_for_key = (*it).second;
        	}

        	if(c >= COLUMNS) node->prev[c-COLUMNS] = node_for_key;
        	else node->next[c] = node_for_key;
        }

        //READ MONTE CARLO VALUES
        ss >> node->evaluation.monte_carlo_values.countWon;
        ss >> node->evaluation.monte_carlo_values.countTotal;

        //READ CONNECT4 STATE
        ss >> node->state.nextPlayer;
        for(int c=0; c<COLUMNS; c++){
        	for(int r=0; r<ROWS; r++){
        		int p;
        		ss >> p;
        		node->state.board[c][r] = p;
        	}
        }

        if(!found_next){
        	//no next means this is a leave
        	//we only consider leaves that aren't winnable
  			//note: this overwrites the monte_carlo values, but the resulting values should be the exact same!
  			int game_state = evaluate_state(node);
        	if(game_state == STATE_PLAYABLE) leaves.insert(node_pair(node->key, node));
        	else if(game_state == STATE_WINNABLE) leavesWinnable.insert(node_pair(node->key, node));
        }else{
        	//if this is not a leave in the loaded graph, make sure that
        	//the current graph doesn't consider this as a leave
        	leaves.erase(node->key);
        	leavesWinnable.erase(node->key);
        }
	}
	unlock_graph();
}


void MonteCarloGraph::save_graph(string filename){
	ofstream graphFile;
	graphFile.open(filename, ofstream::out | ofstream::trunc);
	serialize(graphFile);
	graphFile.close();
	cout << "Graph saved." << endl;
}


void MonteCarloGraph::load_graph(string filename){
	ifstream graphFile;
	graphFile.open(filename, ifstream::in);
	deserialize(graphFile);
	graphFile.close();
	cout << "Graph loaded." << endl;
}