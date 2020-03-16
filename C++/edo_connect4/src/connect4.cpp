
#include <connect4.h>
#include <assert.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <time.h>

using namespace std;

//small helper func
bool in_bounds(int c, int r){
    return (c>=0 && c<COLUMNS) && (r>=0 && r<ROWS);
}


void printMove(connect4_move& move){
    cout << "Move: drop at " << move.column << " (" << move.row;
    cout << ") by player ";
    printPiece(move.player);
}
void printState(connect4_state& state){
    cout << "--- Connect4" << endl << "Next player: ";
    if(state.nextPlayer == RED)
        cout << "Red" << endl;
    else
        cout << "Blue" << endl;
    cout << "-----------------" << endl;
    for(int r = ROWS-1; r>=0; r--){
        cout << "+";
        for(int c = 0; c<COLUMNS; c++){
            char piece_c;
            if(state.board[c][r] == RED) piece_c = 'O';
            else if(state.board[c][r] == BLUE) piece_c = 'X';
            else piece_c = ' ';

            cout << " " << piece_c;
        }
        cout << " +" << endl;
    }
    cout << "-----------------" << endl;
}
void printWinner(int winner){
    switch(winner){
    case RED_WON: cout << "Red won" << endl; break;
    case BLUE_WON: cout << "Blue won" << endl; break;
    case TIE: cout << "Tie" << endl; break;
    case NO_WINNER_YET: cout << "No winner yet" << endl; break;
    default: cout << "Unknown winner type" << endl;
    }
}
void printPiece(int piece){
    switch(piece){
    case RED: cout << "Red" << endl; break;
    case BLUE: cout << "Blue" << endl; break;
    case EMPTY: cout << "Empty" << endl; break;
    default: cout << "Unknown piece/player type" << endl;
    }
}


// ----- CLASS FUNCTIONS -----

Connect4::Connect4(move_strategy s, int startingPlayer){
    for(int c = 0; c<COLUMNS; c++)
        for(int r = 0; r<ROWS; r++)
            currentState.board[c][r] = EMPTY;
    currentState.nextPlayer = startingPlayer;
    piecesDropped = 0;
    strategy = s;
    lastMove.column = lastMove.row = lastMove.player = -1;

    bool load_graph = false;
    if(strategy == MONTE_CARLO_THREADED_LOADED){
        strategy = MONTE_CARLO_THREADED;
        load_graph = true;
    }

    graph = nullptr;
    monteCarloGraph = nullptr;
    if(strategy == GRAPH){
        graph = new GameGraph(currentState);
        graph->explore_to_depth(GRAPH_LIMIT);
    }else if(strategy == MONTE_CARLO || strategy == MONTE_CARLO_THREADED){
        monteCarloGraph = new MonteCarloGraph(currentState);
        monteCarloGraph->explore_to_depth(GRAPH_LIMIT);
    }
    if(strategy == MONTE_CARLO_THREADED){
        if(load_graph) monteCarloGraph->load_graph("graph.dat");
        monteCarloGraph->start_thread();
        monteCarloGraph->start_thread();
    }
}


Connect4::~Connect4(){
    delete graph;
    delete monteCarloGraph;
}

void Connect4::printBoard(){
    printState(currentState);
}

void Connect4::setCurrentState(connect4_state& state){
    currentState = state;
    piecesDropped = 0;
    for(int c = 0; c<COLUMNS; c++){
        for(int r = 0; r<ROWS; r++){
            if(currentState.board[c][r] != EMPTY){
                piecesDropped++;
            }
        }
    }
}

bool Connect4::canDrop(int column){
    return canDrop(column, currentState);
}

void Connect4::dropPiece(int column){
    connect4_move move;
    move.column = column;
    move.player = currentState.nextPlayer;
    move.row = -1;
    executeMove(move);
}

void Connect4::dropPiece(int column, int player){
    int player_temp = currentState.nextPlayer;
    currentState.nextPlayer = player;
    dropPiece(column, currentState);
    currentState.nextPlayer = player_temp;
    piecesDropped++;
}


void Connect4::executeMove(connect4_move& move){
    if(currentState.nextPlayer != move.player){
        throw std::invalid_argument("Wrong player in move!");
    }

    if(strategy == GRAPH){
        int c_mirrored = isMirrored(currentState) ? (COLUMNS-move.column-1) : move.column;
        graph->prune_move(c_mirrored);
    }else if(strategy == MONTE_CARLO || strategy == MONTE_CARLO_THREADED){
        int c_mirrored = isMirrored(currentState) ? (COLUMNS-move.column-1) : move.column;
        monteCarloGraph->prune_move(c_mirrored);
    }

    dropPiece(move.column, currentState);

    piecesDropped++;
    lastMove = move;
}

/* DISABLED until support for graphs is added
void Connect4::correctMove(connect4_move& move, int columnTo){
    currentState.board[move.column][move.row] = EMPTY;
    //drop pieces above, really there should be any but just to make sure
    for(int r=move.row; r<ROWS-1; r++){
        currentState.board[move.column][r] = currentState.board[move.column][r+1];
    }
    currentState.board[move.column][ROWS-1] = EMPTY;

    dropPiece(columnTo, move.player);
    //lastMove.column = columnTo;
}
*/

int Connect4::checkMoveSucceeded(connect4_move& expectedMove, connect4_state& state){
    //currently doesn't check if the state is valid etc.

    if(state.board[expectedMove.column][expectedMove.row] == expectedMove.player)
        return MOVE_SUCCEEDED;

    int column, row;

    for(column=expectedMove.column+1; column<COLUMNS; column++){
        row = highestFreeSpace(column, currentState);
        if(state.board[column][row] == expectedMove.player){
            expectedMove.column = column;
            expectedMove.row = row;
            return WRONG_COLUMN;
        }
    }

    for(column=expectedMove.column-1; column>=0; column--){
        row = highestFreeSpace(column, currentState);
        if(state.board[column][row] == expectedMove.player){
            expectedMove.column = column;
            expectedMove.row = row;
            return WRONG_COLUMN;
        }
    }

    return PIECE_DROPPED;
}


int Connect4::lookForMove(connect4_state& otherState, connect4_move& move){
    //add the move to the Board
    int disimilarities = 0;
    //comare the new state with the currentState
    for(int i = 0; i <= COLUMNS - 1; i++){
        for(int j = 0; j<= ROWS-1; j++){
            if(currentState.board[i][j]!= otherState.board[i][j]){
                move.column = i;
                move.row = j;
                move.player = otherState.board[i][j];
                disimilarities +=1;
            }
        }
    }
    //the player changed more than one pieces
    if(disimilarities > 1) return CHEATING;
    //the user didn't play or the piece fell down
    if(disimilarities == 0) return NO_MOVE_FOUND;
    //check if the player used other color piece
    if(move.player != currentState.nextPlayer) return CHEATING;
    else return MOVE_FOUND;
}


void Connect4::setBoardState(std::vector<std::vector<int>> board){
    if(board.size() != COLUMNS && board[0].size() != ROWS){
        throw std::invalid_argument("Board size doesn't match!");
    }
    piecesDropped = 0;
    for(int c = 0; c<COLUMNS; c++){
        for(int r = 0; r<ROWS; r++){
            currentState.board[c][r] = (uint8_t)board[c][r];
            if(board[c][r] != EMPTY){
                piecesDropped++;
            }
        }
    }
}


int Connect4::isGameOver(){

    //TODO make this more efficient?
    //algorithm: check every position on the board, and go in every direction to see if there are 4 pieces
    //can be made more efficient, as this makes a lot of redundant checks
    for(int c = 0; c<COLUMNS; c++){
        for(int r = 0; r<ROWS; r++){
            int piece = currentState.board[c][r];
            if(piece == EMPTY) continue;

            //go into each direction and check if there are 3 same pieces
            int directions[8][2] = {{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};

            for(int dir=0; dir<8; dir++){
                int c_dir = c;
                int r_dir = r;
                int step = 0;
                for(; step<3; step++){
                    c_dir += directions[dir][0];
                    r_dir += directions[dir][1];

                    if(!in_bounds(c_dir,r_dir)) break;
                    if(currentState.board[c_dir][r_dir] != piece) break;
                }

                //3 extra pieces found! -> game won
                if(step == 3){
                    if(piece == RED) return RED_WON;
                    return BLUE_WON;
                }
            }

        }
    }

    //TODO maybe check if there can be a winner at all instead
    if(piecesDropped == COLUMNS*ROWS)
        return TIE;
    return NO_WINNER_YET;
}

int Connect4::playGame(){
    int winner;
    connect4_move move;
    while((winner=isGameOver()) == NO_WINNER_YET){
        getBestMove(move);
        executeMove(move);
    }
    return winner;
}


void Connect4::getBestMove(connect4_move& move){
    move.player = currentState.nextPlayer;

    //check if there is a winning or urgent move
    connect4_move moveRed, moveBlue;
    int potentialWinner = canWin(currentState, moveRed, moveBlue);
    switch(potentialWinner){
    case RED_WON:
        move = moveRed;
        move.player = currentState.nextPlayer;
        return;
    case BLUE_WON:
        move = moveBlue;
        move.player = currentState.nextPlayer;
        return;
    case TIE:
        move = currentState.nextPlayer == RED ? moveRed:moveBlue;
        return;
    default: break;
    }

    //best move at start: middle
    if(piecesDropped == 0){
        move.column = COLUMNS/2;
        assert(canDrop(move.column)); //we assume the algorithm to get the best move checks this
        move.row = highestFreeSpace(move.column, currentState);
        return;
    }


    //calculate best move
    switch(strategy){
    case RANDOM:
        strategy_random(currentState, move);
        break;
    case MIRROR:
        strategy_mirror(currentState, move);
        break;
    case MIRROR_STATELESS:
        strategy_mirrorStateless(currentState, move);
        break;
    case DEPTH_LIMITED:
        strategy_depthLimited(currentState, move);
        break;
    case GRAPH:
        strategy_graph(currentState, move);
        break;
    case MONTE_CARLO:
        strategy_monteCarlo(currentState, move);
        break;
    case MONTE_CARLO_THREADED:
        strategy_monteCarloThreaded(currentState, move);
        break;
    case Q_LEARNING:
        cout << "Not yet implemented" << endl;
        assert(false);
        break;
    }

    assert(canDrop(move.column)); //we assume the algorithm to get the best move checks this
    move.row = highestFreeSpace(move.column, currentState);
}

void Connect4::preComputeMoves(){
    cout << "Not yet implemented" << endl;
    assert(false);
}

// ----- STRATEGIES -----

void Connect4::strategy_random(connect4_state& state, connect4_move& move){
    int c = std::rand() %COLUMNS;
    int c_init = c;
    while(!canDrop(c, state)){
        c = (c+1) %COLUMNS;
        if(c == c_init) throw std::invalid_argument("State doesn't allow any move!"); //no possible move
    }
    move.column = c;
}

void Connect4::strategy_mirror(connect4_state& state, connect4_move& move){
    move = lastMove;
    move.column = COLUMNS -1 -move.column;
    if(move.player == RED) move.player = BLUE;
    else move.player = RED;
    if(!canDrop(move.column, state)){
        strategy_random(state, move);
    }
}

void Connect4::strategy_mirrorStateless(connect4_state& state, connect4_move& move){
    //get the previous move
    //assumes that moves were mirrored until now, otherwise it might not be mirrored
    for(int c = 0; c<COLUMNS/2; c++){
        for(int r = 0; r<ROWS; r++){
            int c_i = COLUMNS - c - 1;
            int p = state.board[c][r];
            int p_i = state.board[c_i][r];
            if(p == p_i || p == move.player || p_i == move.player) continue;

            //one piece on one place, while the mirrored place is still empty
            if(p != EMPTY && p_i == EMPTY){
                move.column = c_i;
                return;
            }
            if(p == EMPTY && p_i != EMPTY){
                move.column = c;
                return;
            }

        }
    }

    //no mirror move found (first move or opponent played in the middle)
    //-> play in middle, or random if already full
    if(canDrop(COLUMNS/2, state)){
        move.column = COLUMNS/2;
    }else{
        strategy_random(state, move);
    }
}

double Connect4::strategy_depthLimited(connect4_state& state, connect4_move& move, int depth, int MAX_DEPTH){
    if(depth >= MAX_DEPTH){
        return evaluateState(state, move.player);
    }

    //only interested in whether it is possible to win, not the actual moves
    connect4_move temp_move;
    int potentialWinner = canWin(state, temp_move, temp_move);
    if(potentialWinner == move.player || potentialWinner == TIE) return 1000.0;
    else if(potentialWinner != NO_WINNER_YET) return -1000.0;

    vector<connect4_state> reachableStates;
    vector<int> columnDrops;
    getReachableStates(currentState, reachableStates, columnDrops);
    assert(reachableStates.size() == columnDrops.size());
    if(reachableStates.size() == 0) return 0.0;

    double maxValue = -10000000.0;
    int maxIndex = 0;
    for(int i=0; i<reachableStates.size(); i++){
        double value = strategy_depthLimited(reachableStates[i], move, depth+1, MAX_DEPTH);
        if(value > maxValue){
            maxValue = value;
            maxIndex = i;
        }
    }

    move.column = columnDrops[maxIndex];
    return 0;
}

void Connect4::strategy_graph(connect4_state& state, connect4_move& move){
    float evaluation[COLUMNS];
    graph->get_root_evaluation(evaluation);
    float maxValue = -20000.0;
    vector<int> bestColumns;
    bool is_mirrored = isMirrored(state);

    for(int c=0; c<COLUMNS; c++){
        int c_mirrored = is_mirrored? COLUMNS-c-1:c;
        //same value as current best moves
        if(-1e-8 < maxValue-evaluation[c] && maxValue-evaluation[c] < 1e-8){
            if(canDrop(c_mirrored, state)){
                bestColumns.push_back(c_mirrored);
            }
        }else if(maxValue < evaluation[c]){//new best move found
            if(canDrop(c_mirrored, state)){
                maxValue = evaluation[c];
                bestColumns.clear();
                bestColumns.push_back(c_mirrored);
            }
        }
    }

    graph->explore_to_depth(GRAPH_LIMIT);

    //choose random column from the best ones
    assert(bestColumns.size() > 0);
    int choice = std::rand()%bestColumns.size();
    move.column = bestColumns[choice];
}

void Connect4::strategy_monteCarlo(connect4_state& state, connect4_move& move){
    float evaluation[COLUMNS];
    monteCarloGraph->get_root_evaluation(evaluation);
    float maxValue = -20000.0;
    vector<int> bestColumns;
    bool is_mirrored = isMirrored(state);

    for(int c=0; c<COLUMNS; c++){
        int c_mirrored = is_mirrored? COLUMNS-c-1:c;
        //same value as current best moves
        if(-1e-8 < maxValue-evaluation[c] && maxValue-evaluation[c] < 1e-8){
            if(canDrop(c_mirrored, state)){
                bestColumns.push_back(c_mirrored);
            }
        }else if(maxValue < evaluation[c]){//new best move found
            if(canDrop(c_mirrored, state)){
                maxValue = evaluation[c];
                bestColumns.clear();
                bestColumns.push_back(c_mirrored);
            }
        }
    }

    monteCarloGraph->explore_to_depth(GRAPH_LIMIT);
    for(int i=0; i<100; i++)
        monteCarloGraph->monte_carlo_step(50);

    //choose random column from the best ones
    assert(bestColumns.size() > 0);
    int choice = std::rand()%bestColumns.size();
    move.column = bestColumns[choice];
}

void Connect4::strategy_monteCarloThreaded(connect4_state& state, connect4_move& move){
    float evaluation[COLUMNS];
    monteCarloGraph->get_root_evaluation(evaluation);
    float maxValue = -20000.0;
    vector<int> bestColumns;
    bool is_mirrored = isMirrored(state);

    for(int c=0; c<COLUMNS; c++){
        int c_mirrored = is_mirrored? COLUMNS-c-1:c;
        //same value as current best moves
        if(-1e-8 < maxValue-evaluation[c] && maxValue-evaluation[c] < 1e-8){
            if(canDrop(c_mirrored, state)){
                bestColumns.push_back(c_mirrored);
            }
        }else if(maxValue < evaluation[c]){//new best move found
            if(canDrop(c_mirrored, state)){
                maxValue = evaluation[c];
                bestColumns.clear();
                bestColumns.push_back(c_mirrored);
            }
        }
    }

    monteCarloGraph->explore_to_depth(GRAPH_LIMIT);

    //choose random column from the best ones
    if(bestColumns.size() == 0){
        monteCarloGraph->print_graph();
        monteCarloGraph->save_graph();
        for(int c=0; c<COLUMNS; c++) cout << evaluation[c] << endl;
        assert(false);
    }
    int choice = std::rand()%bestColumns.size();
    move.column = bestColumns[choice];
}




// ----- STATIC FUNCTIONS -----

bool Connect4::canDrop(int column, connect4_state& state){
    return highestFreeSpace(column, state) != COLUMN_FULL;
}

void Connect4::dropPiece(int column, connect4_state& state){
    if(!canDrop(column, state)){
        throw std::invalid_argument("Can't drop piece here, column is already full or doesn't exist!");
    }
    int row = highestFreeSpace(column, state);
    state.board[column][row] = state.nextPlayer;

    if(state.nextPlayer == RED)
        state.nextPlayer = BLUE;
    else if(state.nextPlayer == BLUE)
        state.nextPlayer = RED;
    else assert(false); //should never happen
}


bool Connect4::isMirrored(connect4_state& state){
    for(int c=0; c<COLUMNS/2; c++){
        for(int r=0; r<ROWS; r++){
            if(state.board[c][r] == state.board[COLUMNS-c-1][r]) continue;
            return (state.board[COLUMNS-c-1][r] == RED) || 
                   (state.board[COLUMNS-c-1][r] == BLUE && state.board[c][r] == EMPTY);
            if(state.board[c][r] == RED) return false;
            else if(state.board[COLUMNS-c-1][r] == RED) return true;
            else if(state.board[c][r] == BLUE) return false;
            else return true;
        }
    }
    return false;
}

void Connect4::convertMirrored(connect4_state& state){
    for(int c=0; c<COLUMNS/2; c++){
        for(int r=0; r<ROWS; r++){
            uint8_t tmp = state.board[c][r];
            state.board[c][r] = state.board[COLUMNS-1-c][r];
            state.board[COLUMNS-1-c][r] = tmp;
        }
    }
}


int Connect4::highestFreeSpace(int column, connect4_state& state){
    if(column >= COLUMNS || column < 0){
        throw std::invalid_argument("Column value not within range (0, COLUMNS)!");
    }
    for(int r=0; r<ROWS; r++){
        if(state.board[column][r] == EMPTY){
            return r;
        }
    }
    return COLUMN_FULL;
}

uint64_t Connect4::stateToKey(connect4_state& state, bool use_mirrored){
    assert(ROWS == 6 && COLUMNS == 7);
    //encoding:
    // 21 bits: height of each column (3 bit per column)
    // 42 bits: red/blue in each column (6 bit per column)
    uint64_t key = 0;
    for(int c_i=0; c_i<COLUMNS; c_i++){
        int c = use_mirrored ? (COLUMNS-c_i-1) : c_i;
        bool foundEmpty = false;
        for(int r=0; r<ROWS; r++){
            if(state.board[c][r] == EMPTY){
                key += ((uint64_t)r) << (3*c_i);
                foundEmpty = true;
                break;
            }else if(state.board[c][r] == RED){
                key += ((uint64_t)1) << (21 + r + ROWS*c_i);
            }
        }
        if(!foundEmpty) key += ((uint64_t)ROWS) << (3*c_i);
    }

    return key;

}

int Connect4::canWin(connect4_state& state, connect4_move& moveRed, connect4_move& moveBlue){
    bool red_won = false;
    bool blue_won = false;
    //TODO make this more efficient?
    //algorithm: check every position on the board, and go in every direction to see if there are 4 pieces
    //can be made more efficient, as this makes a lot of redundant checks
    for(int c = 0; c<COLUMNS; c++){
        for(int r = 0; r<ROWS; r++){
            int piece = state.board[c][r];
            if(piece == EMPTY) continue;
            //if we already found a winning move, then we can stop searching for this one
            if(piece == RED && red_won) continue;
            if(piece == BLUE && blue_won) continue;

            //go into each direction and check if there are 3 same pieces
            int directions[8][2] = {{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};

            for(int dir=0; dir<8; dir++){
                int c_dir = c;
                int r_dir = r;
                int step = 0;
                bool skipped_one = false;
                int c_win, r_win;
                for(; step<3; step++){
                    c_dir += directions[dir][0];
                    r_dir += directions[dir][1];
                    if(!in_bounds(c_dir,r_dir)) break;
                    if( !skipped_one &&
                        state.board[c_dir][r_dir] == EMPTY &&
                        (!in_bounds(c_dir,r_dir-1)
                            || state.board[c_dir][r_dir-1] != EMPTY) ){
                        //exactly one empty place with something beneath it
                        skipped_one = true;
                        c_win = c_dir;
                        r_win = r_dir;
                    }else if(state.board[c_dir][r_dir] != piece) break;
                }
                //strictly speaking, if skipped one is false, the player already has won
                if(!skipped_one && step == 3) return GAME_ALREADY_OVER;
                if(!skipped_one || step != 3) continue;

                if(piece == RED){
                    moveRed.player = RED;
                    moveRed.column = c_win;
                    moveRed.row = r_win;
                    red_won = true;
                }else{
                    moveBlue.player = BLUE;
                    moveBlue.column = c_win;
                    moveBlue.row = r_win;
                    blue_won = true;
                }

                if(red_won && blue_won) return TIE;

            }

        }
    }

    if(red_won) return RED_WON;
    if(blue_won) return BLUE_WON;
    return NO_WINNER_YET;
}


int Connect4::countWinningMoves(connect4_state& state, int& movesRed, int& movesBlue){
    movesRed = 0;
    movesBlue = 0;
    int c_winRed, c_winBlue;
    c_winRed = c_winBlue = -1;
    //like canWin
    for(int c = 0; c<COLUMNS; c++){
        for(int r = 0; r<ROWS; r++){
            int piece = state.board[c][r];
            if(piece == EMPTY) continue;
            int directions[8][2] = {{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};
            for(int dir=0; dir<8; dir++){
                int c_dir = c;
                int r_dir = r;
                int step = 0;
                bool skipped_one = false;
                int c_win;
                for(; step<3; step++){
                    c_dir += directions[dir][0];
                    r_dir += directions[dir][1];
                    if(!in_bounds(c_dir,r_dir)) break;
                    if( !skipped_one &&
                        state.board[c_dir][r_dir] == EMPTY &&
                        (!in_bounds(c_dir,r_dir-1)
                            || state.board[c_dir][r_dir-1] != EMPTY) ){
                        //exactly one empty place with something beneath it
                        skipped_one = true;
                        c_win = c_dir;
                    }else if(state.board[c_dir][r_dir] != piece) break;
                }
                //strictly speaking, if skipped one is false, the player already has won
                if(!skipped_one && step == 3) return GAME_ALREADY_OVER;
                if(!skipped_one || step != 3) continue;

                //compare to the last winning move if present, we don't count the same one twice
                //only the column matters here for the winning move
                if(piece == RED){
                    if(movesRed == 1 && c_winRed == c_win) continue;
                    movesRed++;
                    c_winRed = c_win;
                }else{
                    if(movesBlue == 1 && c_winBlue == c_win) continue;
                    c_winBlue = c_win;
                    movesBlue++;
                }

                //new: we can't leave, since we have to check for game over
                //old:
                //in these cases we can already leave since there is no useful information left in the state
                //if(state.nextPlayer == RED && (movesBlue == 2 || movesRed == 1)) return;
                //if(state.nextPlayer == BLUE && (movesRed == 2 || movesBlue == 1)) return;
            }
        }
    }
    return 0;
}


void Connect4::getReachableStates(connect4_state& state, vector<connect4_state>& reachableStates, vector<int>&columnDrops){
    for(int c=0; c<COLUMNS; c++){
        connect4_state newState = state;
        if(canDrop(c, state)){
            dropPiece(c, newState);
            reachableStates.push_back(newState);
            columnDrops.push_back(c);
        }
    }
}


double Connect4::evaluateState(connect4_state& state, int player){
    int movesRed, movesBlue;
    countWinningMoves(state, movesRed, movesBlue);
    int movesPlayer = player == RED ? movesRed:movesBlue;
    int movesOpponent = player == BLUE ? movesRed:movesBlue;
    //if you can win/opponent can win
    if(movesPlayer > 0 && state.nextPlayer == player) return 1000.0;
    if(movesOpponent > 0 && state.nextPlayer != player) return -1000.0;

    //else, if winnable next turn
    if(movesPlayer >= 2 && state.nextPlayer != player) return 1000.0;
    if(movesOpponent >= 2 && state.nextPlayer == player) return -1000.0;

    return 100.0*movesPlayer - 100.0*movesOpponent;
}
