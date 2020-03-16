
// This file is used for testing
// These aren't unit tests, these are tests that need to be run and evaluated by hand

//yes I'm cheating
#define private public
#define protected public
#include <connect4.h>
#include <iostream>
#include <cassert>
#include <unistd.h>
#include <fstream>

using namespace std;


void gameStateTest(){
    Connect4 game;
    game.dropPiece(0);
    game.dropPiece(0);
    game.dropPiece(0);
    game.dropPiece(0);
    game.dropPiece(0);
    game.dropPiece(0);
    game.dropPiece(1);
    game.dropPiece(1);
    game.dropPiece(1);
    game.dropPiece(2);
    game.dropPiece(2);
    game.dropPiece(3);
    game.dropPiece(4);
    game.dropPiece(5);
    game.dropPiece(6);
    game.printBoard();
    cout << Connect4::highestFreeSpace(0, game.currentState) << " ";
    cout << Connect4::highestFreeSpace(1, game.currentState) << " ";
    cout << Connect4::highestFreeSpace(2, game.currentState) << " ";
    cout << Connect4::highestFreeSpace(3, game.currentState) << " ";
    cout << Connect4::highestFreeSpace(4, game.currentState) << " ";
    cout << Connect4::highestFreeSpace(5, game.currentState) << " ";
    cout << Connect4::highestFreeSpace(6, game.currentState) << endl;

    connect4_move moveRed, moveBlue;
    printWinner(game.isGameOver());
    printWinner(game.canWin(game.currentState, moveRed, moveBlue));

    game.dropPiece(3);
    game.dropPiece(3);
    game.printBoard();
    printWinner(game.isGameOver());
    printWinner(game.canWin(game.currentState, moveRed, moveBlue));

    game.dropPiece(4);
    game.dropPiece(2);
    game.printBoard();
    printWinner(game.isGameOver());

}

void strategyTest(move_strategy s){
    Connect4 game(s);
    connect4_move move;

    int winner = NO_WINNER_YET;
    while(winner == NO_WINNER_YET){
        game.getBestMove(move);
        printMove(move);
        game.executeMove(move);
        game.printBoard();
        winner = game.isGameOver();
    }

    printWinner(winner);
}

void strategyGraph(){

    Connect4 game(GRAPH);
    connect4_move move;

    int winner = NO_WINNER_YET;
    while(winner == NO_WINNER_YET){
        game.getBestMove(move);
        printMove(move);
        game.executeMove(move);
        game.printBoard();
        winner = game.isGameOver();
    }

    printWinner(winner);
}


void testGraphAll(){
    Connect4 game(GRAPH);
    while(true)
        game.graph->explore_leaves();
}

void testGraphRandomPrune(){
    Connect4 gameG(GRAPH);
    Connect4 gameR(RANDOM);
    connect4_move move;

    while(gameG.isGameOver()==NO_WINNER_YET){
        gameG.graph->explore_leaves();
        gameR.getBestMove(move);
        gameG.executeMove(move);
        gameR.executeMove(move);
        gameG.printBoard();
    }

}

void testGameVS(move_strategy s1, move_strategy s2){
    Connect4 gameG(s1);
    Connect4 gameR(s2);
    connect4_move move;

    while(gameG.isGameOver()==NO_WINNER_YET){
        gameG.getBestMove(move);
        gameG.executeMove(move);
        gameR.executeMove(move);
        gameG.printBoard();

        if(gameG.isGameOver() != NO_WINNER_YET) break;

        gameR.getBestMove(move);
        gameG.executeMove(move);
        gameR.executeMove(move);
        gameG.printBoard();
    }

    printWinner(gameG.isGameOver());
}

void testInteractive(move_strategy s, int startingPlayer=RED){
    srand(time(NULL));
    Connect4 game(s, startingPlayer);
    connect4_move move;

    //decide who starts randomly
    if(rand()%2){
        game.getBestMove(move);
        game.executeMove(move);
        game.printBoard();
    }

    while(game.isGameOver()==NO_WINNER_YET){
        int column;
        move.player = game.currentState.nextPlayer;
        cout << "Your move: ";
        cin >> move.column;
        game.executeMove(move);
        game.printBoard();

        if(game.isGameOver() != NO_WINNER_YET) return;

        game.getBestMove(move);
        game.executeMove(move);
        game.printBoard();
    }

    printWinner(game.isGameOver());
}

void testMonteCarlo(){
    Connect4 game(RANDOM);
    MonteCarloGraph graph(game.currentState);
    srand(55555);
    graph.explore_to_depth(2);
    graph.save_dot();
    graph.monte_carlo_step(10);
    graph.save_dot();
}

void testMonteCarloEval(){
    Connect4 game(RANDOM);
    uint8_t board[ROWS][COLUMNS] = {
        {RED, RED, RED, BLUE, BLUE, RED, BLUE},
        {RED, BLUE, BLUE, BLUE, RED, BLUE, RED},
        {BLUE, BLUE, RED, RED, BLUE, RED, BLUE},
        {RED, RED, BLUE, RED, BLUE, BLUE, BLUE},
        {BLUE, RED, RED, EMPTY, BLUE, RED, RED},
        {EMPTY, BLUE, BLUE, EMPTY, RED, RED, RED}
    };
    for(int c=0; c<COLUMNS; c++)
        for(int r=0; r<ROWS; r++)
            game.currentState.board[c][r] = board[r][c];
    MonteCarloGraph graph(game.currentState);
    srand(55555);
    for(int i=0; i<3; i++) graph.monte_carlo_step(50);
    graph.print_graph();
    graph.save_dot();
}

void testMonteCarloEvalAll(){
    Connect4 game(RANDOM);
    MonteCarloGraph graph(game.currentState);
    srand(55555);
    graph.explore_to_depth(6);
    for(int i=0; i<10000; i++) graph.monte_carlo_step(100);
    graph.save_dot();
    float evaluation[COLUMNS];
    graph.get_root_evaluation(evaluation);
    cout << "Evaluations:" << endl;
    for(int c=0; c<COLUMNS; c++) cout << c << ": " << evaluation[c] << endl;
}

void testLookForMove(){
    Connect4 game(RANDOM);
    uint8_t board[ROWS][COLUMNS] = {
        {RED, RED, RED, BLUE, BLUE, RED, BLUE},
        {RED, BLUE, BLUE, BLUE, RED, BLUE, RED},
        {BLUE, BLUE, RED, RED, BLUE, RED, BLUE},
        {RED, RED, BLUE, RED, BLUE, BLUE, BLUE},
        {BLUE, RED, RED, EMPTY, BLUE, RED, RED},
        {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY}
    };
    for(int c=0; c<COLUMNS; c++)
        for(int r=0; r<ROWS; r++)
            game.currentState.board[c][r] = board[r][c];
    game.printBoard();
    connect4_state otherState = game.currentState;
    connect4_move move;
    int player = game.currentState.nextPlayer;
    int result = game.lookForMove(otherState, move);
    cout << "the result when the piece falls out is:" << result << endl;
    game.dropPiece(0,player);
    result = game.lookForMove(otherState, move);
    cout << "the result when everything is normal is:" << result << endl;
    game.dropPiece(1,player);
    //test more that one piece at once
    result = game.lookForMove(otherState, move);
    cout << "the result when the player plays twice is:" << result << endl;
    if(player ==  RED){
        game.dropPiece(2,BLUE);
        result = game.lookForMove(otherState, move);
        cout << "the result when the player plays other color:" << result << endl;
    }
    if(player ==  BLUE){
        game.dropPiece(2,RED);
        result = game.lookForMove(otherState, move);
        cout << "the result when the player plays other color:" << result << endl;
    }
}

int testCanWin(){
    Connect4 game(RANDOM);
    uint8_t board[ROWS][COLUMNS] = {
        {BLUE, RED, RED, RED, BLUE, RED, BLUE},
        {BLUE, BLUE, BLUE, RED, RED, BLUE, BLUE},
        {BLUE, RED, BLUE, BLUE, BLUE, EMPTY, BLUE},
        {RED, BLUE, BLUE, RED, RED, EMPTY, RED},
        {RED, RED, RED, BLUE, BLUE, EMPTY, RED},
        {RED, RED, BLUE, RED, BLUE, EMPTY, RED}
    };
    for(int c=0; c<COLUMNS; c++)
        for(int r=0; r<ROWS; r++)
            game.currentState.board[c][r] = board[r][c];
    game.currentState.nextPlayer = RED;
    connect4_move move;
    printWinner(Connect4::canWin(game.currentState, move, move));
}

void testCheckMoveSucceeded(){
    Connect4 game(RANDOM);
    uint8_t board[ROWS][COLUMNS] = {
        {RED, RED, RED, RED, BLUE, RED, RED},
        {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY},
        {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY},
        {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY},
        {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY},
        {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY}
    };
    for(int c=0; c<COLUMNS; c++)
        for(int r=0; r<ROWS; r++)
            game.currentState.board[c][r] = board[r][c];
    connect4_state state_correct = game.currentState;
    connect4_state state_dropped = game.currentState;
    connect4_state state_wrong_left = game.currentState;
    connect4_state state_wrong_right = game.currentState;

    int column = 3;
    connect4_move move;
    move.player = RED;
    move.column = column;
    move.row = 1;

    state_correct.board[move.column][move.row] = RED;
    state_wrong_left.board[move.column-1][move.row] = RED;
    state_wrong_right.board[move.column+1][move.row] = RED;

    assert(game.checkMoveSucceeded(move, state_correct) == MOVE_SUCCEEDED);
    assert(game.checkMoveSucceeded(move, state_dropped) == PIECE_DROPPED);
    assert(game.checkMoveSucceeded(move, state_wrong_left) == WRONG_COLUMN);
    assert(move.column == column-1);
    move.column = column;
    assert(game.checkMoveSucceeded(move, state_wrong_right) == WRONG_COLUMN);
    assert(move.column == column+1);

}


void testMCTSSerialization(){
    Connect4 game(RANDOM);
    MonteCarloGraph graph(game.currentState);
    MonteCarloGraph graph2(game.currentState);
    srand(55555);
    graph.explore_to_depth(2);
    for(int i=0; i<10; i++) graph.monte_carlo_step(10);
    graph.save_graph("test_graph.dat");
    graph2.load_graph("test_graph.dat");
    graph.save_dot();
    graph2.save_dot();
}


void testMCTSExploration(){
    Connect4 game(RANDOM);
    MonteCarloGraph graph(game.currentState);

    string filename = "graph_precalc.dat";

    //load if file exists
    ifstream graphFile(filename);
    if(graphFile.good()){
        graphFile.close();
        graph.load_graph(filename);
    }
    graph.explore_to_depth(8);

    for(int i=0; i<4; i++) graph.start_thread();

    int counter = 0;
    while(true){
        sleep(60*30);
        graph.save_graph("checkpoint_"+to_string(counter)+".dat");
        counter++;
        cout << graph.visited_states.size() << " nodes explored" << endl;
    }

}


void testMCTSCorrectness(){
    Connect4 game(RANDOM);
    uint8_t board[ROWS][COLUMNS] = {
        {BLUE, BLUE, RED, RED, BLUE, BLUE, BLUE},
        {RED, BLUE, BLUE, RED, RED, RED, BLUE},
        {RED, BLUE, RED, RED, BLUE, RED, RED},
        {BLUE, EMPTY, RED, BLUE, RED, BLUE, BLUE},
        {EMPTY, EMPTY, BLUE, BLUE, BLUE, RED, RED},
        {EMPTY, EMPTY, RED, BLUE, RED, RED, BLUE}
    };
    for(int c=0; c<COLUMNS; c++)
        for(int r=0; r<ROWS; r++)
            game.currentState.board[c][r] = board[r][c];
    game.currentState.nextPlayer = RED;
    MonteCarloGraph graph(game.currentState);
    srand(55555);

    graph.start_thread();
    while(graph.leaves.size() > 0) sleep(1);
    graph.print_graph();
    graph.save_dot();

    graph.prune_move(1);
    graph.print_graph();
    graph.save_dot();
    graph.explore_to_depth(1);
    graph.prune_move(0);
    graph.print_graph();
    graph.save_dot();

    graph.start_thread();
    while(graph.leaves.size() > 0) sleep(1);
    graph.print_graph();
    graph.save_dot();
}


void testMCTScorrectness2(){
    Connect4 game(RANDOM); 
    /*
    -----------------
    +       O X     +
    +   O X X O     +
    +   X O X X     +
    + O X X X O     +
    + X O X O O O   +
    + X O O O X O   +
    -----------------
    */
    uint8_t board[ROWS][COLUMNS] = {
        {BLUE, RED, RED, RED, BLUE, RED, EMPTY},
        {BLUE, RED, BLUE, RED, RED, RED, EMPTY},
        {RED, BLUE, BLUE, BLUE, RED, EMPTY, EMPTY},
        {EMPTY, BLUE, RED, BLUE, BLUE, EMPTY, EMPTY},
        {EMPTY, RED, BLUE, BLUE, RED, EMPTY, EMPTY},
        {EMPTY, EMPTY, EMPTY, RED, BLUE, EMPTY, EMPTY}
    };
    for(int c=0; c<COLUMNS; c++)
        for(int r=0; r<ROWS; r++)
            game.currentState.board[c][r] = board[r][c];
    game.currentState.nextPlayer = BLUE;
    MonteCarloGraph graph(game.currentState);
    srand(55555);

    for(int i=0; i<10; i++) graph.monte_carlo_step(100);
    graph.print_graph();
    graph.save_dot();

    float evaluation[COLUMNS];
    graph.get_root_evaluation(evaluation);
    for(int c=0; c<COLUMNS; c++){
        cout << evaluation[c] << endl;
    }
}



void testMonteCarloSimple(){
    Connect4 game(RANDOM);
    MonteCarloGraph graph(game.currentState);
    srand(55555);
    for(int i=0; i<5; i++) graph.monte_carlo_step(10);
    graph.print_graph();
    graph.save_dot();
}

int main(int argc, char** argv){
    //testInteractive(MONTE_CARLO_THREADED);
    //testMCTSExploration();
    //testMCTSCorrectness();
    //testMCTScorrectness2();
    //testMonteCarloSimple();

    //gameStateTest();
    //strategyTest(DEPTH_LIMITED);
    //strategyGraph();
    //testGraphAll();
    //testGraphRandomPrune();
    testGameVS(GRAPH, MONTE_CARLO_THREADED);
    //testMonteCarlo();
    //testMonteCarloEval();
    //testMonteCarloEvalAll();
    //testLookForMove();
    //testCanWin();
    //testCheckMoveSucceeded();
    //testMCTSSerialization();
}

