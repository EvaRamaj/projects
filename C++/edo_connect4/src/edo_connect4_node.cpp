#include <connect4.h>
#include <ros/ros.h>
#include <edo_pnp_services.h>
#include <edo_connect4/Connect4State.h>
#include <edo_connect4/PlayerDetector.h>
#include <edo_connect4/ModelUpdate.h>
#include <edo_connect4/Reset.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/SpawnModel.h>
#include <tf/transform_broadcaster.h>
#include <edo_connect4/Dice.h>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include "synthesiser.cpp"
#include "joint_storage.cpp"


int ERR_CHECK_I;
#define ERR_CHECK(x, y) if(ERR_CHECK_I = x){ ROS_ERROR("[Connect4] connect4 client error %i", ERR_CHECK_I); return y; }
#define GRIPPER_OPEN 0.02 // +1.5cm larger than required
#define COLUMN_WIDTH 0.03
#define TOKEN_SIZE 0.0055 // in simulation its .005, in the real world its 0.0049, almost the same

#define SAY(s) if(speak){ say(s); }
#define ASK(s) if(speak){ ask(s); } //if speech is active then so is Dialogflow Speech-to-text

//Variables that describe the state of the client/robot
ros::ServiceClient clientCartesian, clientJoints, clientGripper, clientSpeed, clientSpawn, clientConnect4, clientModel, clientReset, clientPlayer;
tf::Transform pose, poseRobot, posePiece, poseWayPoint, poseBoardToColumn, poseRefill;
tf::Transform poseColumns[COLUMNS];
double currentGripperSpan;
bool sim, edo2, use_camera, speak;
int playerRobot, playerOpponent;

// Will store name and joint [j1,j2..j6] values in radians
std::vector<Joints> measured_joint_vals;

double prev_aggregated_vals;

//variables used to spawn a token
std::string token_red_urdf, token_blue_urdf;
int token_counter = 0; //used to make a unique token name
int counter_picked_up = 0; //used to refill tokens
int counter_piece_dropped = 0; // counter for PIECE_DROPPED
int counter_wrong_column = 0; // counter for WRONG_COLUMN
int counter_cheating = 0; // counter for CHEATING
bool cheated_during_move = false;

//Offsets are relative to pose of the board
double z_offset, y_offset, x_offset;

//forward declare since we're not using a header for this file
int spawn_token(int player, tf::Pose& poseDropPoint);
int get_connect4_state(Connect4& game, connect4_state& state);
int model_update(connect4_move *lastMove);


int sometimes_say(std::string text){
// The probability of speaking is 50%
    int prob = rd()%2;
    if (prob == 0) SAY(text);
    return 0;
}


int seldom_say(std::string text){
// The probability of speaking is 33%
    int prob = rd()%3;
    if (prob == 0) SAY(text);
    return 0;
}


void print_tf(tf::Pose& pose){
    std::cout << "tf Pose: " << std::endl << "{ ";
    std::cout << pose.getOrigin().x() << ", ";
    std::cout << pose.getOrigin().y() << ", ";
    std::cout << pose.getOrigin().z() << "}" << std::endl << "{ ";
    std::cout << pose.getRotation().x() << ", ";
    std::cout << pose.getRotation().y() << ", ";
    std::cout << pose.getRotation().z() << ", ";
    std::cout << pose.getRotation().w() << "}" << std::endl;
}


double height_offset(double gripper_span){
    double res = 4062.17 * (gripper_span*gripper_span) - 44.5706*gripper_span;
    res /= 1000.0;
    // calibrated such that smallest piece is picked correctly using:
    // res * (end+offset)/end - start*offset/end;
    // where:
    // start = 0.000733456; //offset for smallest piece
    // end = 0.0086644; //offset for largest piece
    // offset = 0.005; //how far offset for largest piece needs to be adjusted
    return res*1.577074004 - 0.000423258;
}


int move_tf_pose(tf::Transform& pose){
//these two are copied from edo_pnp, maybe refactor?
    geometry_msgs::Pose gm_pose;
    tf::poseTFToMsg(pose, gm_pose);
    edo_pnp::MoveCartesian srv;
    srv.request.pose = gm_pose;
    if(clientCartesian.call(srv)){
        return 0;
    }
    return 3000;
}


int move_gripper(double gripper_span){
    edo_pnp::MoveGripper srv;
    srv.request.gripper_span = gripper_span;
    if(clientGripper.call(srv)){
        currentGripperSpan = gripper_span;
        return 0;
    }
    return 3001;
}


int move_to_piece(){
    ERR_CHECK(move_tf_pose(posePiece), 901)
    return 0;
}


int pick_up_piece(){
    //first move downer to the piece
    double offset_height = 0.06;
    tf::Transform offset_tf;
    offset_tf.setOrigin(tf::Vector3(0, 0, offset_height));
    offset_tf.setRotation(tf::createQuaternionFromRPY(0,0,0));
    pose = posePiece*offset_tf;

    ERR_CHECK(move_tf_pose(pose), 801)
    //CLOSE THE GRIPPER
    ERR_CHECK(move_gripper(TOKEN_SIZE), 802)
    //GO UP AGAIN
    ERR_CHECK(move_tf_pose(posePiece), 804)

    counter_picked_up++;
    if(counter_picked_up >=3){
        counter_picked_up--;
        ERR_CHECK(spawn_token(playerRobot, poseRefill), 10000)
        ROS_INFO("[Connect4] Refilling 1 token");
    }
    return 0;
}


int command_joint(const Joints& js){

    edo_pnp::MoveJoints jointsSrv;
    jointsSrv.request.joints.j1 = js.j1;
    jointsSrv.request.joints.j2 = js.j2;
    jointsSrv.request.joints.j3 = js.j3;
    jointsSrv.request.joints.j4 = js.j4;
    jointsSrv.request.joints.j5 = js.j5;
    jointsSrv.request.joints.j6 = js.j6;

    int err = clientJoints.call(jointsSrv) ? 0:3008;
    ERR_CHECK(err, 108)
    return 0;
}


int pick_up_piece_js(){

    // move to position above

    // go down

    // close gripper

    // go up again

    return 0;
}


int move_to_waypoint(){
    ERR_CHECK(move_tf_pose(poseWayPoint), 803)
    return 0;
}


int move_to_waypoint_js(){
    return 0;
}


int drop_piece(int column){

    double drop_height = 0.03; //for now it is a predefined value but maybe we should put an offset

    tf::Transform drop_tf, offset_column;
    tf::Pose poseAboveColumn;

    drop_tf.setOrigin(tf::Vector3(0, 0, drop_height));
    drop_tf.setRotation(tf::createQuaternionFromRPY(0,0,0));

    poseAboveColumn = poseColumns[column]*poseBoardToColumn*drop_tf;

    ERR_CHECK(move_tf_pose(poseAboveColumn), 805)
    ERR_CHECK(move_gripper(GRIPPER_OPEN), 702)

    return 0;

}


int drop_piece_js(int column){

    // move to correct column and drop the piece there

    return 0;
}


int execute_move(connect4_move& move){
    ERR_CHECK(move_to_waypoint(), 602)
    ERR_CHECK(drop_piece(move.column), 602)
    ERR_CHECK(move_to_waypoint(), 602)
    ERR_CHECK(move_tf_pose(posePiece), 804)
    ERR_CHECK(pick_up_piece(), 601)
    return 0;
}


int execute_move_js(connect4_move& move){

//    if (joint_vector.size() == 0){
//        ROS_FATAL("The vector containting the joint information is empty. Cannot execute joint movements.");
//    }

    ERR_CHECK(pick_up_piece_js(), 601)
    ERR_CHECK(move_to_waypoint_js(), 602)
    ERR_CHECK(drop_piece_js(move.column), 602)
    ERR_CHECK(move_to_waypoint_js(), 602)
    ERR_CHECK(move_tf_pose(posePiece), 804)
    return 0;
}


int robot_move(Connect4& game){
    connect4_move move, originalMove;
    game.getBestMove(originalMove);
    move = originalMove;

    ERR_CHECK(execute_move(move), 500)

    if(use_camera){
        int moveFound;
        connect4_state visionState;
        do{
            ros::Duration(1.0).sleep();
            model_update(&move);
            get_connect4_state(game, visionState);
            moveFound = game.lookForMove(visionState, move);

            switch(moveFound){
            case NO_MOVE_FOUND:
                ROS_WARN("[Connect4] Piece dropped, retrying!");
                SAY(dropped_a_token());
                counter_piece_dropped++;
                if(counter_piece_dropped >=2){
                    ERR_CHECK(execute_move(move), 501)
                    counter_piece_dropped = 0;
                }else ros::Duration(1.5).sleep();
                break;
            case CHEATING:
                moveFound = MOVE_FOUND;
                move = originalMove;
                cheated_during_move = true;
                break;
            }
        } while(moveFound == NO_MOVE_FOUND);
    }

    if(move.column != originalMove.column){
        ROS_WARN("[Connect4] Piece dropped into wrong column.");
    }
    game.executeMove(move);
    return 0;
}


int spawn_token(int player, tf::Pose& poseDropPoint){
    if(!sim) return 0;
    std::string token_urdf = token_red_urdf;
    if(player == BLUE) token_urdf = token_blue_urdf;

    geometry_msgs::Pose poseMsg;
    tf::poseTFToMsg(poseDropPoint, poseMsg);

    gazebo_msgs::SpawnModel srv;

    srv.request.initial_pose = poseMsg;
    srv.request.reference_frame = "edo_base_link";
    srv.request.robot_namespace = "/edo";
    srv.request.model_xml = token_urdf;
    srv.request.model_name = "token_" + std::to_string(token_counter++);

    if(clientSpawn.call(srv)){
        return 0;
    }
    return 4000;
}


double angle_between_vectors(tf::Vector3& vec1, tf::Vector3& vec2){
//calculates angle in radians between 2D vectors (only x and y axis)
    double dot = vec1.x() * vec2.x() + vec1.y() * vec2.y();
    double det = vec1.x() * vec2.y() - vec1.y() * vec2.x();
    return atan2(det, dot);
}


int move_to_crane(){
// Moves to hover/crane-like position over the game
    edo_pnp::MoveJoints jointsSrv;
    tf::Vector3 xAxis(1,0,0);
    jointsSrv.request.joints.j1 = angle_between_vectors(xAxis, posePiece.getOrigin());
    jointsSrv.request.joints.j2 = 0.2;
    jointsSrv.request.joints.j3 = 1.2;
    jointsSrv.request.joints.j4 = 0.;
    jointsSrv.request.joints.j5 = 180*DEG_TO_RAD - (0.2+1.2);
    jointsSrv.request.joints.j6 = -35*DEG_TO_RAD;

    ERR_CHECK(move_gripper(GRIPPER_OPEN), 103)
    int err = clientJoints.call(jointsSrv) ? 0:3003;
    ERR_CHECK(err, 102)

    return 0;
}


int move_to_sad(){
// Moves to this position after the other player has cheated for the second time
    // TODO: Make it look more depressed than the crane
    edo_pnp::MoveJoints jointsSrv;
    tf::Vector3 xAxis(1,0,0);
    jointsSrv.request.joints.j1 = angle_between_vectors(xAxis, poseColumns[3].getOrigin()) + M_PI;
    jointsSrv.request.joints.j2 = 0.6;
    jointsSrv.request.joints.j3 = 0.6;
    jointsSrv.request.joints.j4 = 0.;
    jointsSrv.request.joints.j5 = 1.8;
    jointsSrv.request.joints.j6 = 2.3708;

    ERR_CHECK(move_gripper(GRIPPER_OPEN), 103)
    int err = clientJoints.call(jointsSrv) ? 0:3003;
    ERR_CHECK(err, 102)

    return 0;
}


int opponent_move(Connect4& game){
    connect4_move move;
    counter_cheating = 0;

    //sometimes_say(remind_its_your_turn());
    seldom_say(joke());
    bool cheated = false;

    if(use_camera && !sim){
        int moveFound;
        connect4_state visionState;
        do{
            ros::Duration(1.0).sleep();
            model_update(nullptr);
            get_connect4_state(game, visionState);
            moveFound = game.lookForMove(visionState, move);

            if(moveFound == CHEATING){
                ROS_WARN("[Connect4] Possible cheating.");
                game.printBoard();
                printState(visionState);
                moveFound = NO_MOVE_FOUND;
                counter_cheating++;


                if(counter_cheating == 3 && !cheated) SAY(detected_cheating());

                if(counter_cheating >= 5 && !cheated){
                    //TODO here cheating is "officially" detected
                    ROS_ERROR("[Connect4] Cheating detected!");
                    SAY(not_playing_anymore());
                    counter_cheating = 0;
                    //move_to_sad();
                    cheated = true;
                }else ros::Duration(3).sleep();
            }
        } while(moveFound == NO_MOVE_FOUND);
        game.executeMove(move);
    }else{
        move.player = game.getNextPlayer();
        while(true){
            std::cout << "Your move (enter column 0-6): ";
            std::cin >> move.column;
            try{
                game.executeMove(move);
                break;
            }catch(...){
                std::cout << "Invalid move!" << std::endl;
            }
        }
    }


    if(sim) spawn_token(move.player, poseColumns[move.column]);
    return 0;
}


int play_connect4(int startingPlayer){

    ros::NodeHandle nh("edo_pnp_services");
    ros::NodeHandle nh_gazebo("/gazebo");
    clientCartesian = nh.serviceClient<edo_pnp::MoveCartesian>("move_cartesian");
    clientJoints = nh.serviceClient<edo_pnp::MoveJoints>("move_joints");
    clientGripper = nh.serviceClient<edo_pnp::MoveGripper>("move_gripper");
    clientSpeed = nh.serviceClient<edo_pnp::SetSpeed>("set_speed");
    if(sim) clientSpawn = nh_gazebo.serviceClient<gazebo_msgs::SpawnModel>("spawn_urdf_model");
    ROS_INFO("Waiting for services...");
    clientCartesian.waitForExistence();
    clientJoints.waitForExistence();
    clientGripper.waitForExistence();
    clientSpeed.waitForExistence();
    if(sim) clientSpawn.waitForExistence();
    ROS_INFO("Starting now!");

    ERR_CHECK(move_to_crane(), 999)
    ERR_CHECK(move_tf_pose(posePiece), 997)
    ERR_CHECK(pick_up_piece(), 601)

    std::cout << "Should have said some other random sentance" << std::endl;


    //TODO play the game
    Connect4 game(MONTE_CARLO_THREADED, startingPlayer);
    int winner = NO_WINNER_YET;

    while(winner == NO_WINNER_YET){

        if(game.getNextPlayer() == playerRobot){
            ERR_CHECK(robot_move(game), 100)

        }else{
            ERR_CHECK(opponent_move(game), 101)
            ros::Duration(3.0).sleep();
        }

        game.printBoard();
        if(cheated_during_move){
            cheated_during_move = false;
        }else{
            winner = game.isGameOver();
        }
    }
    switch(winner){
    case TIE:
        ROS_INFO("Game ended in a tie.");
        SAY(draw_game());
        break;
    case RED_WON:
        ROS_INFO("Red won!");
        if (playerRobot == RED) { SAY(won_game()) }
        else { SAY(lost_game()) }
        break;
    case BLUE_WON:
        if (playerRobot == BLUE) { SAY(won_game()) }
        else { SAY(lost_game()) }
        ROS_INFO("Blue won!");
        break;
    }


    return 0;
}


bool game_tfs(){

    tf::TransformListener listener;
    ros::NodeHandle n("~");

    //offset for game board
    //offset z and x can be combined into one
    tf::Transform offset_board_z;
    offset_board_z.setOrigin(tf::Vector3(0, 0, 0.24 + z_offset));
    offset_board_z.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    tf::Transform offset_board_x;
    offset_board_x.setOrigin(tf::Vector3(x_offset, -0.01 + y_offset, 0));
    offset_board_x.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    tf::Transform offset_board_rot_x;
    offset_board_rot_x.setOrigin(tf::Vector3(0, 0, 0));
    offset_board_rot_x.setRotation(tf::createQuaternionFromRPY(195*DEG_TO_RAD, 0, 0));
    tf::Transform offset_board_rot_z;
    offset_board_rot_z.setOrigin(tf::Vector3(0, 0, 0));
    offset_board_rot_z.setRotation(tf::createQuaternionFromRPY(0, 0, 45*DEG_TO_RAD));

    //offset above piece
    tf::Transform offset_piece;
    offset_piece.setOrigin(tf::Vector3(x_offset, y_offset, 0.06 + z_offset)); // We move 7cm above the
    offset_piece.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    tf::Transform offset_piece_rot;
    offset_piece_rot.setOrigin(tf::Vector3(0, 0, 0));
    offset_piece_rot.setRotation(tf::createQuaternionFromRPY(180*DEG_TO_RAD, 0, 45*DEG_TO_RAD));

    //offset to refill piece
    tf::Transform offset_refill;
    offset_refill.setOrigin(tf::Vector3(x_offset, 0.20, 0.06 + z_offset)); // We move 7cm above the
    offset_refill.setRotation(tf::createQuaternionFromRPY(0, 0, 90*DEG_TO_RAD));

    tf::StampedTransform temp;

    try {
        auto now = ros::Time::now();

        std::string board_link = "/board_connect_4_base_link";
        std::string piece_link = "/board_connect_4_pick_red";
        if(playerRobot == BLUE && sim){
            board_link = "/board_connect_4_base_link_blue";
            piece_link = "/board_connect_4_pick_blue";
        }

        // Pose to game board
        listener.waitForTransform(tf::getPrefixParam(n)+"/edo_base_link", board_link, now, ros::Duration(3.));
        listener.lookupTransform(tf::getPrefixParam(n)+"/edo_base_link", board_link, now, temp);
        poseBoardToColumn = offset_board_x*offset_board_rot_x*offset_board_rot_z;
        tf::Pose poseBoard = temp*offset_board_z;
        poseWayPoint = poseBoard*poseBoardToColumn;
        print_tf(poseBoard);
        for(int column=0; column<COLUMNS; column++){
            //columns: 0 1 2 3 4 5 6
            //we need: -3 -2 -1 0 1 2 3
            //x axis goes the other way, hence -multiplier
            int column_middle = COLUMNS/2;
            int multiplier = column_middle - column;
            tf::Transform offset_column;
            offset_column.setOrigin(tf::Vector3(-multiplier*COLUMN_WIDTH, 0, 0));
            offset_column.setRotation(tf::createQuaternionFromRPY(0,0,0));
            poseColumns[column] = poseBoard*offset_column;
        }

        // Pose to pick up piece place
        listener.waitForTransform(tf::getPrefixParam(n)+"/edo_base_link", piece_link, now, ros::Duration(3.));
        listener.lookupTransform(tf::getPrefixParam(n)+"/edo_base_link", piece_link, now, temp);
        posePiece = temp*offset_piece*offset_piece_rot;
        poseRefill = temp*offset_refill;

        return true;

    }catch (tf::TransformException& ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return false;
}


int get_connect4_state(Connect4& game, connect4_state& state){
    ROS_INFO("Getting connect4 state...");
    edo_connect4::Connect4State srv;
    srv.request.sim = sim;
    if(!clientConnect4.call(srv)){
        ROS_ERROR("[Connect4] Connect4 service failed!");
        return -6;
    }

    for(int c=0; c<COLUMNS; c++)
        for(int r=0; r<ROWS; r++)
            state.board[c][r] = srv.response.board[c*ROWS + r];
    return 0;
}


int model_update(connect4_move *lastMove){
    ROS_INFO("Updating model...");
    edo_connect4::ModelUpdate srv;
    srv.request.last_move_column = -1;
    srv.request.last_move_row = -1;
    srv.request.last_move_player = playerOpponent;
    if(lastMove != nullptr){
        srv.request.last_move_column = lastMove->column;
        srv.request.last_move_row = lastMove->row;
        srv.request.last_move_player = lastMove->player;
    }
    if(!clientModel.call(srv)){
        ROS_ERROR("[Connect4] ModelUpdate service failed!");
        return -8;
    }
    return 0;
}


int reset_vision(){
    ROS_INFO("Resetting vision...");
    edo_connect4::Reset srv;
    if(!clientReset.call(srv)){
        ROS_ERROR("[Connect4] Reset service failed!");
        return -111;
    }
    return 0;
}


int get_player_detector(){
    ROS_INFO("Detecting Player...");
    edo_connect4::PlayerDetector srv;
    if(!clientPlayer.call(srv)){
        ROS_ERROR("[Connect4] Player Detector service failed!");
        throw "the player detector service failed";
    }
    return srv.response.player;
}


unsigned int get_first_player(){
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<edo_connect4::Dice>("roll_dice");
    edo_connect4::Dice srv;
    srv.request.num_markers = 6;
    std::cout << "The first player is ";
    unsigned int first_player;

    if (client.call(srv)){
        unsigned int response = srv.response.first_player;
        if (response == BLUE){
            first_player = BLUE;
            std::cout << "BLUE." << std::endl;
        }
        else {
            first_player = RED;
            std::cout << "RED." << first_player << std::endl;
        }
        return first_player;
    }
    else {
        ROS_ERROR("Failed to call service roll_dice");
        throw "get_first_player failed";
    }
}


auto cartesian_to_joint_measurement(){


    ROS_INFO("About to record joint states. Will go step-by-step through the board.");
    SAY(stretching());

    // Move to HOVER OVER RED PIECE
    /*
    Joints joint_hover_red_p;
    joint_hover_red_p.name = "hover_red_p";
    joint_hover_red_p.j1 = current_joint_vals.j1;
    joint_hover_red_p.j2 = current_joint_vals.j2;
    joint_hover_red_p.j3 = current_joint_vals.j3;
    joint_hover_red_p.j4 = current_joint_vals.j4;
    joint_hover_red_p.j5 = current_joint_vals.j5;
    joint_hover_red_p.j6 = current_joint_vals.j6;


    Joints joint_hover_blue_p;
    joint_hover_blue_p.name = "hover_blue_p";
    joint_hover_blue_p.j1 = 0;
    joint_hover_blue_p.j2 = 0;
    joint_hover_blue_p.j3 = 0;
    joint_hover_blue_p.j4 = 0;
    joint_hover_blue_p.j5 = 0;
    joint_hover_blue_p.j6 = 0;


    Joints joint_pick_red_p;
    joint_pick_red_p.name = "pick_red_p";
    joint_pick_red_p.j1 = current_joint_vals.j1;
    joint_pick_red_p.j2 = current_joint_vals.j2;
    joint_pick_red_p.j3 = current_joint_vals.j3;
    joint_pick_red_p.j4 = current_joint_vals.j4;
    joint_pick_red_p.j5 = current_joint_vals.j5;
    joint_pick_red_p.j6 = current_joint_vals.j6;


    Joints joint_pick_blue_p;
    joint_pick_blue_p.name = "pick_blue_p";
    joint_pick_blue_p.j1 = 0;
    joint_pick_blue_p.j2 = 0;
    joint_pick_blue_p.j3 = 0;
    joint_pick_blue_p.j4 = 0;
    joint_pick_blue_p.j5 = 0;
    joint_pick_blue_p.j6 = 0;


    Joints joint_col_1_way;
    joint_col_1_way.name = "col_1_way";
    joint_col_1_way.j1 = current_joint_vals.j1;
    joint_col_1_way.j2 = current_joint_vals.j2;
    joint_col_1_way.j3 = current_joint_vals.j3;
    joint_col_1_way.j4 = current_joint_vals.j4;
    joint_col_1_way.j5 = current_joint_vals.j5;
    joint_col_1_way.j6 = current_joint_vals.j6;

    Joints joint_col_1;
    joint_col_1.name = "col_1";
    joint_col_1.j1 = current_joint_vals.j1;
    joint_col_1.j2 = current_joint_vals.j2;
    joint_col_1.j3 = current_joint_vals.j3;
    joint_col_1.j4 = current_joint_vals.j4;
    joint_col_1.j5 = current_joint_vals.j5;
    joint_col_1.j6 = current_joint_vals.j6;

    */
    return measured_joint_vals;

}


auto load_joint_navigation(){

    if (board_moved()){

        //navigate thru the board and get all of the joint values for every pose
        measured_joint_vals = cartesian_to_joint_measurement();

        //store all of the joint values to a file for next time
        joints_2_file(measured_joint_vals);

        //store the new board position to file for next time
        board_2_file();
    }

    else measured_joint_vals = get_stored_joint_vals();

    return measured_joint_vals;
}

int dialog_status = 0;
void dialog_callback(const std_msgs::String::ConstPtr& msg){
    //TODO set dialog_status to 1 if yes, to 2 if no, or leave it if no answer yet
    if(msg->data == "no") dialog_status = 2;
    else if (msg->data == "yes") dialog_status = 1;
}


bool play_again(){

    //todo: maybe make this a paramter for the launch file?
    bool disabled = false;
    if(!speak) disabled = true;

    bool play_again = false;

    if(!disabled){
        SAY(wanna_play_again());

        // DIALOGFLOW ASK() and fit into the boolean
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/dialogflow_results", 1000, dialog_callback);
        dialog_status = 0;

        do{
            ros::spinOnce();
        }while(dialog_status == 0);
        play_again = dialog_status == 1;
    }

    if (play_again){

        SAY(please_clear_board());
        ros::Duration(15).sleep();
        return true;
    }

    else {
        SAY(goodbye());
        return false;
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "edo_connect4_node");
    ros::start();

    bool blue_team, dice;
    ros::param::param<bool>("sim", sim, true);
    ros::param::param<bool>("edo2", edo2, false);
    ros::param::param<bool>("use_camera", use_camera, true);
    ros::param::param<bool>("blue_team", blue_team, false);
    ros::param::param<bool>("dice", dice, false);
    ros::param::param<bool>("speak", speak, false);
    ros::param::param<std::string>("player_name", player_name, "friend");

    std::string token_red_path, token_blue_path;
    ros::param::param<std::string>("token_red_path", token_red_path, "");
    ros::param::param<std::string>("token_blue_path", token_blue_path, "");
    std::ifstream red_file(token_red_path);
    std::ifstream blue_file(token_blue_path);
    token_red_urdf = std::string((std::istreambuf_iterator<char>(red_file)), std::istreambuf_iterator<char>());
    token_blue_urdf = std::string((std::istreambuf_iterator<char>(blue_file)), std::istreambuf_iterator<char>());

    if (speak){
        ros::NodeHandle nh_converse(""); //TODO: Will handle the listener as well
        clientSpeak = nh_converse.serviceClient<edo_connect4::EdoSpeak>("edo_speak");
        clientSpeak.waitForExistence();
    }

    if(use_camera){
        ros::NodeHandle nh_vision("edo_connect4_services");
        clientConnect4 = nh_vision.serviceClient<edo_connect4::Connect4State>("get_connect4_state");
        clientConnect4.waitForExistence();
        clientModel = nh_vision.serviceClient<edo_connect4::ModelUpdate>("model_update");
        clientModel.waitForExistence();
        clientReset = nh_vision.serviceClient<edo_connect4::Reset>("reset");
        clientReset.waitForExistence();
	    clientPlayer = nh_vision.serviceClient<edo_connect4::PlayerDetector>("player_detector");
	    clientPlayer.waitForExistence();
    }

    if(sim){ //sim config
        x_offset = 0.;
        y_offset = 0.;
        z_offset = 0.;
    }else if(edo2){ //edo2 config
        x_offset = 0.004;
        y_offset = -0.005;
        z_offset = 0.001;
    }else{ //edo1 config
        x_offset = -0.0075;
        y_offset = -0.004;
        z_offset = 0.003;
    }

    //load_joint_navigation();

    SAY(greetings());

    do {

    	reset_vision();

        // Find out who the 1st player to move will be
        unsigned int startingPlayer;
        if(dice and use_camera){
            SAY(lets_roll_the_dice());
            startingPlayer = get_first_player();
        }else{
            startingPlayer = RED;
        }

        if(use_camera){
            playerRobot = get_player_detector();
            if(playerRobot == RED) ROS_INFO("Red player!");
            else ROS_INFO("Blue player!");
        }else{
            playerRobot = blue_team ? BLUE : RED;
        }
        playerOpponent = playerRobot == RED ? BLUE : RED;

        SAY(announce_first_player(startingPlayer, playerRobot));

        //get tfs
        while ( not game_tfs() ) {
            ROS_INFO("Waiting for transforms from the game.");
            ros::Duration(1).sleep();
        }

        int err = play_connect4(startingPlayer);
        if (err) return 55555;

    } while (play_again() and speak);

    return 0;
}
