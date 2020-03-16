
#include <connect4.h>
#include <iostream>

using namespace std;


void gameInteractive(move_strategy s){
    srand(time(NULL));
    Connect4 game(s);
    connect4_move move;
    game.printBoard();

    //decide who starts randomly
    if(rand()%2){
        game.getBestMove(move);
        game.executeMove(move);
        game.printBoard();
    }

    while(game.isGameOver()==NO_WINNER_YET){
        move.player = game.getNextPlayer();
        while(true){
            cout << "Your move (enter column 0-6): ";
            cin >> move.column;
            try{ 
                game.executeMove(move); 
                break;
            }catch(...){
                cout << "Invalid move!" << endl;
            }
        }
        game.printBoard();

        if(game.isGameOver() != NO_WINNER_YET) break;

        game.getBestMove(move);
        game.executeMove(move);
        game.printBoard();
    }

    printWinner(game.isGameOver());
}


int main(int argc, char** argv){
    gameInteractive(MONTE_CARLO_THREADED);
}

