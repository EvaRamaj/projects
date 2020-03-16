
#ifndef PROJECT_EDOHANOI_H
#define PROJECT_EDOHANOI_H


#include <ros/ros.h>
#include <edo_pnp_services.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <vector>
#include <iostream>
#include <cstdint>


//the enum and the array are aligned!!
enum hanoi_piece {
    TOKEN_200,
    TOKEN_240,
    TOKEN_295,
    TOKEN_340,
    TOKEN_375,
    TOKEN_395,
    TOKEN_460,
    TOKEN_520,
    TEMP1, //placeholder tokens
    TEMP2,
    TEMP3,
    TEMP4,
    TEMP5,
    TEMP6,
    TEMP7,
    TEMP8,
    TEMP9,
    TEMP10,
    TEMP11,
    TEMP12
};

static const double piece_size[] = {
    0.02,
    0.024,
    0.0295,
    0.034,
    0.0375,
    0.0395,
    0.046,
    0.052,
    0.0521, //placeholder tokens
    0.0522,
    0.0523,
    0.0524,
    0.0525,
    0.0526,
    0.0527,
    0.0528,
    0.0529,
    0.05291,
    0.05292,
    0.05293
};

#define HEIGHT_SIZE 0.0064 // less than 1cm tall each

//rod identifiers, could also be declared as enum
#define ROD_LEFT 0
#define ROD_MID 1
#define ROD_RIGHT 2

//Note: height/rod is strictly speaking redundant information, but it makes calculations easier
//height refers to the index of the piece counting upwards, i.e. 0 mean the piece is at the bottom
//1 means the piece lies on the piece at the bottom etc.


/* Defines a move for the robot:
 * piece - the piece to move (one of the PIECE_X from above)
 * rodFrom/rodTo - move the piece from rodFrom to rodTo (ROD_XXX from above)
 * heightFrom - height that the piece is currently at as an index, 0 means the piece at the bottom, 1 the one above etc.
 */
typedef struct hanoi_move{
    hanoi_piece piece; 
    int rodFrom, rodTo, heightFrom;
} hanoi_move;


/* Defines a hanoi game state. 3 list are saved, the index corresponds to the definitions of the rods above.
 * Each list contains the pieces, where the first element in the list is at the bottom.
 */
typedef struct hanoi_state{
    std::vector<hanoi_piece> rods[3];
} hanoi_state;

typedef struct Point2i{
    int x,y;
} Point2i;


//helper struct only for solveHanoi
typedef struct graph_node{
    hanoi_state state;
    hanoi_move move;
    int nodePrev;
    int expectedSize; //used to init the vector with correct size, basically depth/length of solution
} graph_node;


class Hanoi
{
public:
    Hanoi();
    Hanoi(hanoi_state& initialState);
    ~Hanoi();

    int getPieceAt(int rod, int height);
    int getHeightOfPiece(hanoi_piece piece);
    int getRodOfPiece(hanoi_piece piece);
    Point2i getPosOfPiece(hanoi_piece);

    //checks if you can place a piece at the rod
    bool canPlace(hanoi_piece piece, int rod);
    //adds the piece at rod
    void addPiece(hanoi_piece piece, int rod);
    //moves the piece at the top of rodFrom to rodTo
    void moveTopPiece(int rodFrom, int rodTo);

    //returns a list of moves that solve hanoi
    void solveHanoi(std::vector<hanoi_move>& moves);
    void solveHanoi(Hanoi& goalHanoi, std::vector<hanoi_move>& moves);

    void printHanoi();
    void printState(hanoi_state& state);
    void printMoves(std::vector<hanoi_move>& moves, bool printResultingState=false);

private:
    hanoi_state currentState;

    //like above, but saves the state to the given reference instead of the current state
    bool canPlace(hanoi_state& state, hanoi_piece piece, int rod);
    void addPiece(hanoi_piece piece, int rod, hanoi_state& state);
    void moveTopPiece(int rodFrom, int rodTo, hanoi_state& state);

    //compares two states
    bool compareStates(hanoi_state& state1, hanoi_state& state2);

    //not used anymore
    bool isInList(hanoi_state& state, std::vector<graph_node> list);
    
    //this function assumes a maximum of 20 hanoi pieces
    uint64_t stateToKey(hanoi_state& state);

};

#endif //PROJECT_EDOHANOI_H


