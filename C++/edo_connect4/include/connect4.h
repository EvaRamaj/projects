
#ifndef PROJECT_EDOCONNECTFOUR_H
#define PROJECT_EDOCONNECTFOUR_H

#include <vector>
#include <inttypes.h>

//pieces and players
#define EMPTY 0
#define RED 1
#define BLUE 2

//game state
#define NO_WINNER_YET 10
#define TIE 11
#define RED_WON 12
#define BLUE_WON 13
#define GAME_ALREADY_OVER 14

//looking for moves
#define NO_MOVE_FOUND 0
#define MOVE_FOUND 2
#define CHEATING 1 

//check move succeded
#define MOVE_SUCCEEDED 100 
#define PIECE_DROPPED 101
#define WRONG_COLUMN 102

//board size
#define COLUMNS 7
#define ROWS 6

#define COLUMN_FULL ROWS

//used by graph search strategies
#define DEPTH_LIMIT 5
#define GRAPH_LIMIT 8

#define NUM_JOINTS 6

class GameGraph;
class MonteCarloGraph;

//strategies to use for the AI
enum move_strategy{
    RANDOM,
    MIRROR,
    MIRROR_STATELESS,
    DEPTH_LIMITED,
    GRAPH,
    MONTE_CARLO,
    MONTE_CARLO_THREADED,
    MONTE_CARLO_THREADED_LOADED,
    Q_LEARNING,
};

static const char* waypoint_names[] = {
        "hover_red_p",
        "hover_blue_p",
        "pick_red_p",
        "pick_blue_p",
        "col_1_way",
        "col_2_way",
        "col_3_way",
        "col_4_way",
        "col_5_way",
        "col_6_way",
        "col_7_way",
        "col_1",
        "col_2",
        "col_3",
        "col_4",
        "col_5",
        "col_6",
        "col_7",
};


typedef struct connect4_move {
    int player; //RED or BLUE
    int column; //0-COLUMNS
    int row; //0-ROWS, not really needed
} connect4_move;

typedef struct connect4_state {
    uint8_t board[COLUMNS][ROWS]; //columns go from left (0) ro right (COLUMNS), rows go from down (0) to up (ROWS)
    int nextPlayer; //next player to place a piece
} connect4_state;

void printMove(connect4_move& move);
void printState(connect4_state& state);
void printWinner(int winner);
void printPiece(int piece);

class Connect4
{
    friend GameGraph;
    friend MonteCarloGraph;
public:
    Connect4() : Connect4(DEPTH_LIMITED) {}
    Connect4(move_strategy s, int startingPlayer=RED);
    ~Connect4();

    int getNextPlayer(){ return currentState.nextPlayer; }
    connect4_move getLastMove(){ return lastMove; }

    void setCurrentState(connect4_state& state);

    void printBoard();

    //returns true if you can drop the piece at the specified column (row max!)
    bool canDrop(int column);

    //drops the piece at the column and updates game state (including player!)
    //use executeMove instead whenever possible
    void dropPiece(int column);

    //basically a wrapper for dropPiece, you can use this or dropPiece
    void executeMove(connect4_move& move);

    //DISABLED as it doesn't support graph systems
    //corrects the move to the given column, should be called when the robot
    //made an error when placing a piece
    //void correctMove(connect4_move& move, int columnTo);

    //checks if move succeeded, note that this function isn't doing en extensive check over the whole game state
    //returns MOVE_SUCCEEDED, PIECE_DROPPED, WRONG_COLUMN accordingly and writes the actual move into expectedMove
    int checkMoveSucceeded(connect4_move& expectedMove, connect4_state& state);

    //compares current state with otherState, and stores the move between the state and returns MOVE_FOUND
    //if no valid move was found, it will return NO_MOVE_FOUND if it didn't find one, and CHEATING if it was a cheating move
    int lookForMove(connect4_state& otherState, connect4_move& move);

    //sets the whole game board state to the given board
    //can be used if expected state (from game logic) and actual state (from vision) are too different
    void setBoardState(std::vector<std::vector<int>> board);

    //returns the game state, see defines above
    int isGameOver();
    //plays the game until the end, returns the winner, used for testing and in monte carlo search
    int playGame();

    //calculates the best move for the current player
    void getBestMove(connect4_move& move);

    //maybe later for optimization: pre-compute moves while opponent is making a choice
    void preComputeMoves();


private:
    //current game state
    connect4_state currentState;
    connect4_move lastMove;
    int piecesDropped;
    move_strategy strategy;
    GameGraph *graph;
    MonteCarloGraph *monteCarloGraph;


    //false if most (or equal) pieces are on the left half
    static bool isMirrored(connect4_state& state);
    //mirrors the given state
    static void convertMirrored(connect4_state& state);
    //returns the row index in the specified column which is the first free space
    //or COLUMN_FULL if the column is full
    static int highestFreeSpace(int column, connect4_state& state);
    //compresses state to 64 bit integer
    static uint64_t stateToKey(connect4_state& state, bool use_mirrored=false);

    //like above, but uses the state given as parameter
    static bool canDrop(int column, connect4_state& state);
    static void dropPiece(int column, connect4_state& state);

    //checks if it is possible to win from the given state
    //if so, returns the RED_WON/BLUE_WON/TIE and writes the winning move into the corresponding move-structs
    //else returns NO_WINNER_YET
    static int canWin(connect4_state& state, connect4_move& moveRed, connect4_move& moveBlue);

    //counts how many moves exist where red/blue would win
    //similar to canWin, implementation is mostly copy pasted
    //result is written to the two integers passed
    //returns GAME_ALREADY_OVER if the game is already won, else 0
    static int countWinningMoves(connect4_state& state, int& movesRed, int& movesBlue);

    //fills list with all reachable states from given state
    static void getReachableStates(connect4_state& state, std::vector<connect4_state>& reachableStates, std::vector<int>& columnDrops);

    //evaluates the current board for the given player
    //returns a score that is larger the more beneficial the state is for the player
    static double evaluateState(connect4_state& state, int player);


    //like above, but allows to set the player and doesn't toggle current player
    void dropPiece(int column, int player);

    //---STRATEGIES to get best move
    //plays randomly
    void strategy_random(connect4_state& state, connect4_move& move);

    //always tries to copy opponent
    void strategy_mirror(connect4_state& state, connect4_move& move);
    void strategy_mirrorStateless(connect4_state& state, connect4_move& move);

    //explores state graph using BFS until depth limit is reached
    //IMPORTANT: assumes that the winning player is move.player (needs to be already set!)
    double strategy_depthLimited(connect4_state& state, connect4_move& move, int depth=0, int MAX_DEPTH=DEPTH_LIMIT);

    //creates a state graph and keeps track of it
    void strategy_graph(connect4_state& state, connect4_move& move);
    void strategy_monteCarlo(connect4_state& state, connect4_move& move);
    void strategy_monteCarloThreaded(connect4_state& state, connect4_move& move);

    //TODO other strategies
};


//this header needs all of the above :/
#include <connect4_graph.h>

#endif //PROJECT_EDOCONNECTFOUR_H
