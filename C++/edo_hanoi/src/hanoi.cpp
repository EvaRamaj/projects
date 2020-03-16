
#include <hanoi.h>
#include <cassert>
#include <algorithm>
#include <unordered_set>

Hanoi::Hanoi(){
}

Hanoi::Hanoi(hanoi_state& initialState){
    currentState = initialState;
}

Hanoi::~Hanoi(){
}

int Hanoi::getPieceAt(int rod, int height){
    assert(currentState.rods[rod].size() > height);
    return currentState.rods[rod][height];
}

int Hanoi::getHeightOfPiece(hanoi_piece piece){
    for(int i=0; i<3; i++){
        for(int k=0; k<currentState.rods[i].size(); k++){
            if(currentState.rods[i][k] == piece)
                return k;
        }
    }
    assert(false);
    return -1;
}

int Hanoi::getRodOfPiece(hanoi_piece piece){
    for(int i=0; i<3; i++){
        for(int k=0; k<currentState.rods[i].size(); k++){
            if(currentState.rods[i][k] == piece)
                return i;
        }
    }
    assert(false);
    return -1;
}

Point2i Hanoi::getPosOfPiece(hanoi_piece piece){
    for(int i=0; i<3; i++){
        for(int k=0; k<currentState.rods[i].size(); k++){
            if(currentState.rods[i][k] == piece)
                return {i,k};
        }
    }
    assert(false);
    return {-1, -1};
}

bool Hanoi::canPlace(hanoi_piece piece, int rod){
    return canPlace(currentState, piece, rod);
}

bool Hanoi::canPlace(hanoi_state& state, hanoi_piece piece, int rod){
    if(state.rods[rod].size() == 0) return true;
    hanoi_piece pieceBelow = state.rods[rod].back();
    return piece_size[pieceBelow] > piece_size[piece];
}

void Hanoi::addPiece(hanoi_piece piece, int rod){
    addPiece(piece, rod, currentState);
}

void Hanoi::addPiece(hanoi_piece piece, int rod, hanoi_state& state){
    assert(canPlace(state, piece, rod));
    state.rods[rod].push_back(piece);
}

void Hanoi::moveTopPiece(int rodFrom, int rodTo){
    moveTopPiece(rodFrom, rodTo, currentState);
}

void Hanoi::moveTopPiece(int rodFrom, int rodTo, hanoi_state& state){
    assert(state.rods[rodFrom].size() > 0);
    hanoi_piece piece = state.rods[rodFrom].back();
    state.rods[rodFrom].pop_back();
    addPiece(piece, rodTo, state);
}

void Hanoi::printHanoi(){
    std::cout << "Hanoi ---" << std::endl;
    printState(currentState);
}

void Hanoi::printState(hanoi_state& state){
    std::cout << "Left rod: ";
    for(int i=0; i<state.rods[0].size(); i++) std::cout << state.rods[0][i] << " ";
    std::cout << std::endl << "Middle rod: ";
    for(int i=0; i<state.rods[1].size(); i++) std::cout << state.rods[1][i] << " ";
    std::cout << std::endl << "Right rod: ";
    for(int i=0; i<state.rods[2].size(); i++) std::cout << state.rods[2][i] << " ";
    std::cout << std::endl;
}

void Hanoi::printMoves(std::vector<hanoi_move>& moves, bool printResultingState){
    std::cout << "Hanoi Moves ---" << std::endl;
    hanoi_state state = currentState;
    if(printResultingState){
        std::cout << "Printing states. Note that this assumes the state of this hanoi game hasn't changed since calling solveHanoi." << std::endl;
        std::cout << "--------- Starting in state: " << std::endl;
        printState(state);
        std::cout << "---------" << std::endl;
    }
    for(int i=0; i<moves.size(); i++){
        std::cout << "Move piece " << moves[i].piece;
        std::cout << " (at height " << moves[i].heightFrom;
        std::cout << ") from rod " << moves[i].rodFrom;
        std::cout << " to rod " << moves[i].rodTo;
        std::cout << "." << std::endl;
        if(printResultingState){
            std::cout << "---------" << std::endl;
            moveTopPiece(moves[i].rodFrom, moves[i].rodTo, state);
            printState(state);
            std::cout << "---------" << std::endl;
        }
    }
}

bool Hanoi::compareStates(hanoi_state& state1, hanoi_state& state2){
    for(int i=0; i<3; i++){
        if(state1.rods[i].size() != state2.rods[i].size()) return false;
        for(int k=0; k<state1.rods[i].size(); k++){
            if(state1.rods[i][k] != state2.rods[i][k]) return false;
        }
    }
    return true;
}

uint64_t Hanoi::stateToKey(hanoi_state& state){
    //basic idea:
    //20 bits per rod
    //bit X of those 20 bits is set 1 if piece X is on that rod
    //-> defines unique identifier for max. 20 pieces
    uint64_t key = 0;
    for(int i=0; i<3; i++){
        int shifter = 20*i;
        for(int k=0; k<state.rods[i].size(); k++){
            key += ((uint64_t)1) << (shifter+state.rods[i][k]);
        }
    }
    return key;
}


bool Hanoi::isInList(hanoi_state& state, std::vector<graph_node> list){
    for(int i=0; i<list.size(); i++){
        if(compareStates(list[i].state, state)) return true;
    }
    return false;
}


void Hanoi::solveHanoi(std::vector<hanoi_move>& moves){
    Hanoi goalHanoi;
    std::vector<hanoi_piece> pieces;
    //gather all pieces
    for(int i=0; i<3; i++){
        for(int k=0; k<currentState.rods[i].size(); k++){
            pieces.push_back(currentState.rods[i][k]);
        }
    }

    //add largest piece one after another
    while(pieces.size() > 0){
        hanoi_piece piece = pieces[0];
        double max_size = piece_size[piece];
        int index = 0;
        for(int i=1; i<pieces.size(); i++){
            if(piece_size[pieces[i]] > max_size){
                piece = pieces[i];
                max_size = piece_size[piece];
                index = i;
            }
        }
        pieces.erase(pieces.begin()+index);
        goalHanoi.addPiece(piece, ROD_LEFT);
    }

    solveHanoi(goalHanoi, moves);
}

void Hanoi::solveHanoi(Hanoi& goalHanoi, std::vector<hanoi_move>& moves){
    moves.clear();
    hanoi_state goalState = goalHanoi.currentState;

    //this implements BFS on the state space of the hanoi game

    std::vector<graph_node> frontier;
    hanoi_move noMove;
    frontier.push_back({currentState, noMove, -1, 0});

    std::unordered_set<uint64_t> visitedStates;
    visitedStates.insert(stateToKey(currentState));

    //loop continues until end of list and no new nodes were added to the list
    for(int i=0; i<frontier.size(); i++){
        graph_node node = frontier[i];
        hanoi_state state = node.state;
        int size = node.expectedSize;
        //we can ignore this rod, since we would either move the token back,
        //or move it to the third rod, which is already explored by the previous node
        int previousRod = node.move.rodTo;

        //reachable states: move top piece of each rod
        for(int k=0; k<3; k++){
            if(k == previousRod || state.rods[k].size() == 0) continue;
            hanoi_piece piece = state.rods[k].back();

            //try to move to other rods
            for(int j=1; j<3; j++){
                int otherRod = (k+j)%3;
                if(!canPlace(state, piece, otherRod)) continue;

                hanoi_state nextState = state; //copy!
                moveTopPiece(k, otherRod, nextState); //move the piece
                if(visitedStates.count(stateToKey(nextState)) > 0) continue;

                //new possible state found!
                hanoi_move move = {piece, k, otherRod, (int)nextState.rods[k].size()};
                if(!compareStates(nextState, goalState)){
                    //state isn't goal, add to frontier
                    frontier.push_back({nextState, move, i, size+1});
                    visitedStates.insert(stateToKey(nextState));
                }else{
                    //reached goal -> go backwards and fill moves list
                    moves.reserve(size+1);
                    moves.push_back(move);
                    int nodePrev = i;
                    if(nodePrev == -1) return; //only one move
                    while(true){
                        node = frontier[nodePrev];
                        nodePrev = node.nodePrev;
                        if(nodePrev == -1) break;
                        moves.push_back(node.move);
                    }
                    std::reverse(moves.begin(), moves.end());
                    return;
                }
            }
        }
    }
    std::cout << "No solution found?" << std::endl;
    assert(false);
}


