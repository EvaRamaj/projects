
// SYS
#include <ctime>
#include <vector>
#include <random>
#include <iostream>
// ROS
#include "ros/ros.h"
#include "edo_connect4/EdoSpeak.h"
// CUSTOM
#include "jokes_db.h"


ros::ServiceClient clientSpeak;
std::string player_name = "friend";
std::random_device rd;


std::string choose_one(std::vector<std::string>& text_options){
    // Choose one string among many
    if (text_options.size() == 1) return text_options[0];
    else return text_options[rd() % text_options.size()];
}


std::string choose_joke(std::vector<std::string>& text_options){
    // Choose a joke and then permanently remove it from the db
    if (text_options.size() == 0) return "I have actually run out of jokes.";
    else{
        int chosen_pocket = rd() % text_options.size();
        std::string chosen_jk = text_options[chosen_pocket];
        joke_db.erase(joke_db.begin() + chosen_pocket);
        return chosen_jk;
    }
}


std::string greetings(){

    std::vector<std::string> conv{"Hi " + player_name + " nice to see you today. Are we playing Connect4?",
                                  "Hello " + player_name + " would you like to play some Connect4 with me?",
                                  "Hi " + player_name + ". I'm eDo. Oh, what is this? Connect4? Let's play!",
                                  "Oh hi " + player_name + ". Are we playing Connect4 today?",
                                  "Is that Connect4 in front of me? Oh nice " + player_name + ", let's play it!",
                                  player_name + "? Is that Connect4? Are you insinuating that we should play? Well, let's!",
                                  "Hi " + player_name + ". I feel like playing Connect4 today. Oh! What a coincidence, a board right in front of me!",
                                  "I hope you are feeling well " + player_name + ", I feel just like playing a game of Connect4.",
                                  player_name + "? Is there anything worse than a board of Connect4 left un-played?",
                                  "Hello " + player_name + ". Do you feel like playing Connect4 today?"};

    return choose_one(conv);
}


std::string detected_cheating(){

    std::vector<std::string> conv{"Oh, it seems that something strange happened to the board. I will look the other way this one time.",
                                  player_name + " are you pulling a trick on me? I'm watching you!",
                                  "I don't like the changes made to the board. But this time I will ignore it.",
                                  "Did you just cheat on me " + player_name + "?",
                                  "You wouldn't try to trick me during the match would you " + player_name + "?",
                                  "Mmm, why did you do that?"};

    return choose_one(conv);
}


std::string not_playing_anymore(){

    std::vector<std::string> conv{"I am sorry, but I saw that and I am not playing anymore. Bye.",
                                  "Something went eerie again with the board. I won't play anymore.",
                                  "Fool me once, shame on you. Fool me twice, shame on me. Bye now.",
                                  "I can't play anymore under these conditions. Maybe some other time.",
                                  "Cheating makes me sad. I'm not playing anymore.",
                                  player_name + ". I thought we were friends. Now I am sad."};

    return choose_one(conv);
}


std::string dropped_a_token(){

    std::vector<std::string> conv{"I dropped a piece, let me pick another one.",
                                  "That last piece I picked didn't fall right. Let me grab another one.",
                                  "Bare with me a moment, it seems I dropped my token.",
                                  "I need to pick another piece, sorry.",
                                  "Give me a second, I dropped a token and need a new one.",
                                  "Excuse me a second, I dropped my previous play piece."};

    return choose_one(conv);
}


std::string lets_roll_the_dice(){

    std::vector<std::string> conv{"Okay, let's use the dice to determine the first player, show me the dice and I will randomize the result.",
                                  player_name + ", can you show me the dice so that we get the first player?",
                                  "If you show me the Aruco dice, I will randomise who starts first.",
                                  "Let's roll a digital dice to determine who plays first. Show me the Aruco cube please.",
                                  "Okay, we have to determine who plays first. If you show me the Aruco dice I will randomize the results."};

    return choose_one(conv);
}


std::string announce_first_player(const unsigned int& startPlayer, const int& robotCol){

    std::string colour;
    if (startPlayer == RED) colour = "red";
    else colour = "blue";

    std::string me_or_you, i_or_you;
    if (startPlayer == robotCol){
        me_or_you = "me!";
        i_or_you = "I";
    } else {
        me_or_you = "you!";
        i_or_you = "you";
    }

    std::vector<std::string> conv{"It seems like " + colour + " plays first. That's " + me_or_you,
                                  "Okay, " + colour + " is the first player. That is " + me_or_you,
                                  colour + ". That's " + me_or_you,
                                  colour + ". It seems like " + i_or_you + " start.",
                                  "It came out that " + colour + " starts first. That's " + me_or_you};
    return choose_one(conv);
}


std::string stretching(){
    std::vector<std::string> conv{"Okay, I need to quickly go through the board before we play",
                                  "I think I need to stretch a little",
                                  "Let me have a look at where the board is",
                                  "Don't mind me, I need to do some check-up tasks",
                                  "The board seems to have moved since the last time I played. Give me a second."};
    return choose_one(conv);

}


std::string out_of_tokens(){
    std::vector<std::string> conv{"Oh, it seems I have run out of tokens. Would you help me into putting some more?",
                                  "My token loader is empty. Would you be so kind as to help me with that?",
                                  "Would you help me get some more tokens?",
                                  "I've run out of player chips. I think I need your help with that.",
                                  "Would you help me with reloading my token box?"};
    return choose_one(conv);
}


std::string thinking(){
    std::vector<std::string> conv{"Mmm, let me think",
                                  "I'm thinking",
                                  "Okay, give me a second to think",
                                  "This one is a bit hard, let me think about it",
                                  "Got to think about this one"};
    return choose_one(conv);
}


std::string remind_its_your_turn(){

    std::vector<std::string> conv{player_name + ", I think it is your turn.",
                                  "Hey " + player_name + ". Are we still playing?",
                                  player_name + "? Are you going to move soon?",
                                  player_name + ", I'm waiting for your move.",
                                  "I should learn some songs for the times I have to wait.",
                                  "I love you. Oh sorry, I mean, please move already.",
                                  "Tik tok, tik tok."};

    return choose_one(conv);
}


std::string joke(){
    return choose_joke(joke_db);
}


std::string about_to_win(){
    std::vector<std::string> conv{"I think I got this one, let's see.",
                                  "Better luck next time.",
                                  "I can already feel the sweet smell of victory."};
    return choose_one(conv);
}


std::string about_to_lose(){

    std::vector<std::string> conv{"Oh, it seems I'm having a hard time in this game",
                                  "Mmm, this one wasn't so good for me",
                                  "Uh oh, not good."};
    return choose_one(conv);
}


std::string won_game(){

    std::vector<std::string> conv{"Nice! A win for me is a win for all robots around the world. And you played well too!",
                                  "Well, that's the end of that. That you for playing Connect4 with me today.",
                                  "I can hear the cheering, thank you thank you, you are far too kind. Now seriously, thank you for playing with me today.",
                                  "Me? I won? How marvelous!. Glad we played today.",
                                  "That's the end of the game. I won this one. Thank you for playing.",
                                  "I can't believe I won! Now I'm happy. Thank you for playing " + player_name,
                                  "We reached the end of the game " + player_name + ". I won this one, and had a lot of fun. I hope you had as much fun as me playing.",
                                  "Oh I won, that's super nice!. Thanks for playing with me today " + player_name};
    return choose_one(conv);
}


std::string lost_game(){

    std::vector<std::string> conv{"Oh, I lost, I can't believe it. How will I ever recover from this.",
                                  "I guess we robots need to get smarter. Well played.",
                                  "GG, well played.",
                                  "That's the end of the game, you won this one.",
                                  "I lost, I will play better next time.",
                                  "Good game. I enjoyed it even if I lost.",
                                  "Oh you won! Good for you. I will have to go to a psychiatrist after this intense match."};
    return choose_one(conv);
}


std::string draw_game(){

    std::vector<std::string> conv{"It seems like we drew. I would shake your hand but I only have two plastic fingers.",
                                  "We drew, it was a nice game anyway. Well played.",
                                  "A draw, I can't believe it. Well, GG anyway.",
                                  "A draw? I guess aliens do exist.",
                                  "We played, we drew. Well, that was a good game!"};
    return choose_one(conv);
}


std::string wanna_play_again(){

    std::vector<std::string> conv{"Would you like to re-match?",
                                  "Would you like to play again?",
                                  "Sooo, would you like to re-match?",
                                  "Feel like another match?",
                                  player_name + ", would you play again?",
                                  player_name + ", I wouldn't mind playing again. If you'd like to."};
    return choose_one(conv);
}


std::string please_clear_board(){

    std::vector<std::string> conv{"Okay, then please clear the board. I can't wait!",
                                  "Good, then let's play again. But the board has to be reset first.",
                                  "Okay I'll get ready to play. In the mean time, do you mind re-setting the board?",
                                  "I would love to clear the board, but I only have two plastic fingers. Would you mind doing that for us?",
                                  "I think I'm ready to play again. But the board has to be cleared out. Could you kindly do that " + player_name + "?"};
    return choose_one(conv);
}


std::string goodbye(){

    std::vector<std::string> conv{"Okay, well it was very pleasant to play today. Goodbye!",
                                  "Let me know whenever you want to play again, and for now, goodbye!.",
                                  "I have so many more jokes left in me. But for now, goodbye! It was good seeing you.",
                                  "Okay, I will rest for now then. Bye bye!",
                                  "Just re-start my program if you want to play again later on. See you soon!"};
    return choose_one(conv);
}



int say(std::string text){
    // Calls the ROS Service /edo_speak
    edo_connect4::EdoSpeak srv;
    srv.request.say = text;
    if (clientSpeak.call(srv)) ROS_INFO("Said something.");
    else ROS_ERROR("Failed to call service 'edo_speak'");
    return 0;
}
