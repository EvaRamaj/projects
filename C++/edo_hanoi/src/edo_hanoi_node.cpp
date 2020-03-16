
#include <hanoi.h>
#include <edo_hanoi/HanoiState.h>
#include <ctime>

int ERR_CHECK_I;
#define ERR_CHECK(x, y) if(ERR_CHECK_I = x){ ROS_ERROR("[Hanoi] hanoi client error %i", ERR_CHECK_I); return y; }
//#define ERR_CHECK(x, y) if(x != 0){ ROS_ERROR("[Hanoi] hanoi client error: %i", y); return x; }
#define GRIPPER_OPEN 0.067 // 8mm larger than the biggest disk

ros::ServiceClient clientCartesian, clientJoints, clientGripper, clientSpeed;
tf::Transform pose, tower_left, tower_center, tower_right, edo_offset;
double currentGripperSpan;
bool sim, edo2;

//Values that are different in sim/real world:
//For simulation: hardcoded values
//For real robot: simulation adjusted for height_offset by gripper_span
double z_offset;
double piece_height;


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

    // piece-wise linear version
    /*
    double offset_width;
    double offset_start;
    double offset = 0.0;
    if(gripper_span < 0.01){
        offset_width = 0.2;
        offset_start = 0.0;
        offset = (gripper_span-0.00)/0.01 * offset_width + offset_start;
    }else if(gripper_span < 0.02){
        offset_width = 0.84;
        offset_start = 0.2;
        offset = (gripper_span-0.01)/0.01 * offset_width + offset_start;
    }else if(gripper_span < 0.03){
        offset_width = 1.4;
        offset_start = 1.04;
        offset = (gripper_span-0.02)/0.01 * offset_width + offset_start;
    }else if(gripper_span < 0.04){
        offset_width = 2.34;
        offset_start = 2.44;
        offset = (gripper_span-0.03)/0.01 * offset_width + offset_start;
    }else if(gripper_span < 0.05){ 
        offset_width = 3.07;
        offset_start = 4.78;
        offset = (gripper_span-0.04)/0.01 * offset_width + offset_start;
    }else if(gripper_span < 0.055){ 
        offset_width = 2.05;
        offset_start = 7.85;
        offset = (gripper_span-0.05)/0.005 * offset_width + offset_start;
    }else{ //< 0.06
        offset_width = 2.4;
        offset_start = 9.9;
        offset = (gripper_span-0.055)/0.005 * offset_width + offset_start;
    }

    return offset/1000.0;*/
}


//these two are copied from edo_pnp, maybe refactor?
int move_tf_pose(tf::Transform& pose){
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


// Moves gripper above rod
int move_to_rod(int rod){

    // Set the rotation to be facing down
    tf::Transform poseDown;
    poseDown.setOrigin(tf::Vector3(0, 0, 0));
    poseDown.setRotation(tf::createQuaternionFromRPY(180*DEG_TO_RAD, 0, 135*DEG_TO_RAD));

    if (rod == ROD_RIGHT){
        pose = tower_right*poseDown;
    }

    if (rod == ROD_MID){
        pose = tower_center*poseDown;
    }

    if (rod == ROD_LEFT){
        pose = tower_left*poseDown;
    }

    ERR_CHECK(move_tf_pose(pose), 901)
    return 0;
}


// Note: 0.10122 is the lowest piece and 0.1416 is the highest piece
int pick_up_piece(int height, double size){

    //offset height to the piece
    //this is basically the constant (0.205 - (0.102+0.0064*1.75)) minus an offset implied by the piece to be picked
    double offset_height = 0.0918 - height*piece_height;
    if(!sim){
        offset_height -= height_offset(currentGripperSpan) - height_offset(size);
    }

    // Lower the gripper
    tf::Transform offset_tf;
    offset_tf.setOrigin(tf::Vector3(0, 0, offset_height));
    offset_tf.setRotation(tf::createQuaternionFromRPY(0,0,0));
    pose = pose*offset_tf;

    ERR_CHECK(move_tf_pose(pose), 801)

    // Close the gripper
    //loosen grip for the simulation
    if(sim) size += 0.003;
    else size -= 0.0055;
    ERR_CHECK(move_gripper(size), 802)
    //wait, sometimes the library doesn't wait until the gripper's closed
    //ros::Duration(0.5).sleep(); 

    // Move up again
    offset_tf.setOrigin(tf::Vector3(0, 0, -offset_height));
    offset_tf.setRotation(tf::createQuaternionFromRPY(0,0,0));
    pose = pose*offset_tf;

    ERR_CHECK(move_tf_pose(pose), 803)

    return 0;
}


//simply drop the piece
int drop_piece(){

    double drop_height = z_offset - 0.165;

    tf::Transform drop_tf;
    drop_tf.setOrigin(tf::Vector3(0, 0, drop_height));
    drop_tf.setRotation(tf::createQuaternionFromRPY(0,0,0));
    pose = pose*drop_tf;
    ERR_CHECK(move_tf_pose(pose), 701)

    ERR_CHECK(move_gripper(GRIPPER_OPEN), 702)

    drop_tf.setOrigin(tf::Vector3(0, 0, -drop_height));
    drop_tf.setRotation(tf::createQuaternionFromRPY(0,0,0));
    pose = pose*drop_tf;
    ERR_CHECK(move_tf_pose(pose), 703)

    return 0;

}


int execute_move(hanoi_move& move){
    ERR_CHECK(move_to_rod(move.rodFrom), 601)
    ERR_CHECK(pick_up_piece(move.heightFrom, piece_size[move.piece]), 602)
    ERR_CHECK(move_to_rod(move.rodFrom), 603)
    ERR_CHECK(move_to_rod(move.rodTo), 604)
    ERR_CHECK(drop_piece(), 605)
    return 0;
}

// TODO: Find out why this is not working
// Moves the robot to the very starting pose, where all joints are 0
int move_to_vertical(){
    edo_pnp::MoveJoints jointsSrv;
    jointsSrv.request.joints.j1 = 0.f;
    jointsSrv.request.joints.j2 = 0.f;
    jointsSrv.request.joints.j3 = 0.f;
    jointsSrv.request.joints.j4 = 0.f;
    jointsSrv.request.joints.j5 = 0.f;
    jointsSrv.request.joints.j6 = 0.f;
    int err = clientJoints.call(jointsSrv) ? 0:3002;
    ERR_CHECK(err, 101)
    return 0;
}


//calculates angle in radians between 2D vectors (only x and y axis)
double angle_between_vectors(tf::Vector3& vec1, tf::Vector3& vec2){
    double dot = vec1.x() * vec2.x() + vec1.y() * vec2.y();
    double det = vec1.x() * vec2.y() - vec1.y() * vec2.x();
    return atan2(det, dot);
}


// Moves to hover/crane-like position over the game
int move_to_crane(){
    edo_pnp::MoveJoints jointsSrv;
    tf::Vector3 xAxis(1,0,0);
    jointsSrv.request.joints.j1 = angle_between_vectors(xAxis, tower_center.getOrigin());
    jointsSrv.request.joints.j2 = 0.87;
    jointsSrv.request.joints.j3 = 0.6;
    jointsSrv.request.joints.j4 = 0.;
    jointsSrv.request.joints.j5 = 1.67;
    jointsSrv.request.joints.j6 = 2.3708;

    ERR_CHECK(move_gripper(GRIPPER_OPEN), 103)
    int err = clientJoints.call(jointsSrv) ? 0:3003;
    ERR_CHECK(err, 102)

    return 0;
}


int solve_hanoi(std::vector<hanoi_move>& solutionMoves){

    ros::NodeHandle nh("/edo/edo_pnp_services");
    clientCartesian = nh.serviceClient<edo_pnp::MoveCartesian>("move_cartesian");
    clientJoints = nh.serviceClient<edo_pnp::MoveJoints>("move_joints");
    clientGripper = nh.serviceClient<edo_pnp::MoveGripper>("move_gripper");
    clientSpeed = nh.serviceClient<edo_pnp::SetSpeed>("set_speed");
    ROS_INFO("Waiting for services...");
    clientCartesian.waitForExistence();
    clientJoints.waitForExistence();
    clientGripper.waitForExistence();
    clientSpeed.waitForExistence();
    ROS_INFO("Starting now!");

    ERR_CHECK(move_to_crane(), 999)

    for(int i=0; i<solutionMoves.size(); i++){
        ERR_CHECK(execute_move(solutionMoves[i]), 1000+i)
    }
    return 0;
}


bool game_tfs(){

    tf::TransformListener listener;

    //our offset for the pose above the rods
    //x_offset is not zero different for the real robot!
    //z_offset-0.101 chosen s.t. in simulation the robot is 0.104 above the rod
    tf::Transform offset_tower;
    offset_tower.setOrigin(tf::Vector3(0, 0, z_offset-0.101));
    offset_tower.setRotation(tf::createQuaternionFromRPY(0, 0, 0));

    tf::StampedTransform temp_tower;

    try {

        auto now = ros::Time::now();

        // Pose to towers_of_hanoi_tower_LEFT
        listener.waitForTransform("/edo/edo_base_link", "/towers_of_hanoi_tower_left", now, ros::Duration(3.));
        listener.lookupTransform("/edo/edo_base_link", "/towers_of_hanoi_tower_left", now, temp_tower);
        tower_left = temp_tower*offset_tower;

        // Pose to towers_of_hanoi_tower_CENTER
        listener.waitForTransform("/edo/edo_base_link", "/towers_of_hanoi_tower_center", now, ros::Duration(3.));
        listener.lookupTransform("/edo/edo_base_link", "/towers_of_hanoi_tower_center", now, temp_tower);
        tower_center = temp_tower*offset_tower;

        // Pose to towers_of_hanoi_tower_RIGHT
        listener.waitForTransform("/edo/edo_base_link", "/towers_of_hanoi_tower_right", now, ros::Duration(3.));
        listener.lookupTransform("/edo/edo_base_link", "/towers_of_hanoi_tower_right", now, temp_tower);
        tower_right = temp_tower*offset_tower;

        if(sim){
            tower_left = edo_offset*tower_left;
            tower_center = edo_offset*tower_center;
            tower_right = edo_offset*tower_right;
        }

        return true;

    }

    catch (tf::TransformException& ex){

        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    return false;

}

int get_hanoi_state(Hanoi& hanoi, int tokens, bool sim){
    ROS_INFO("Getting hanoi state...");
    ros::NodeHandle nh("/edo/edo_hanoi_services");
    auto clientHanoi = nh.serviceClient<edo_hanoi::HanoiState>("get_hanoi_state");
    clientHanoi.waitForExistence();
    edo_hanoi::HanoiState srv;
    srv.request.tokens = tokens;
    srv.request.sim = sim;
    if(!clientHanoi.call(srv)){
        ROS_ERROR("[Hanoi] Hanoi service failed!");
        return -6;
    }

    int counter = 0;

    for(int i=0; i<8; i++){
        if(srv.response.state[i] == -1) continue;
        if(!hanoi.canPlace(static_cast<hanoi_piece>(7-i), srv.response.state[i])){
            ROS_ERROR("[Hanoi] Piece cannot be placed, invalid hanoi state.");
            return -7;
        }
        hanoi.addPiece(static_cast<hanoi_piece>(7-i), srv.response.state[i]);
        counter++;
    }

    if(counter != tokens){
        ROS_ERROR("Invalid amount of tokens returned by hanoi state service!");
        ROS_ERROR("%i vs %i", tokens, counter);
        return -1000;
    }

    return 0;
}



int main(int argc, char** argv){
    ros::init(argc, argv, "edo_hanoi_node");
    ros::start();

    int tokens;
    bool use_camera;
    ros::param::param<bool>("sim", sim, true);
    ros::param::param<int>("tokens", tokens, 8);
    ros::param::param<bool>("edo2", edo2, false);
    ros::param::param<bool>("use_camera", use_camera, true);

    if(sim){ //sim config
        z_offset = 0.205;
        piece_height = 0.0064;
        //not used in sim, but just to make sure: set it to zero
        edo_offset.setOrigin(tf::Vector3(0,0,0));
        edo_offset.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    }else if(edo2){ //edo2 config
        //0.0087 = 0.0064*1.75 -0.0025
        z_offset = 0.2137 - height_offset(GRIPPER_OPEN);
        piece_height = 0.007;
        edo_offset.setOrigin(tf::Vector3(0.0045,0,0));
        edo_offset.setRotation(tf::createQuaternionFromRPY(0, 0, 30.4*DEG_TO_RAD));
    }else{ //edo1 config
        //0.0117 = 0.0064*1.75 -0.0025 +0.003
        z_offset = 0.205 - height_offset(GRIPPER_OPEN) + 0.0117 - 0.002;
        piece_height = 0.007;
        edo_offset.setOrigin(tf::Vector3(0.0075,0,0));
        edo_offset.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
    }

    if(tokens > 8 || tokens < 1){
        std::cout << "Only 1-8 tokens allowed!" << std::endl;
        return -2;
    }


    // ----- HANOI SETUP
    Hanoi hanoi;
    //fill hanoi by placing pieces like this:
    /*
    hanoi.addPiece(TOKEN_520, ROD_LEFT);
    hanoi.addPiece(TOKEN_460, ROD_LEFT);
    hanoi.addPiece(TOKEN_375, ROD_LEFT);
    hanoi.addPiece(TOKEN_340, ROD_LEFT);
    hanoi.addPiece(TOKEN_295, ROD_LEFT);
    hanoi.addPiece(TOKEN_240, ROD_LEFT);
    hanoi.addPiece(TOKEN_200, ROD_LEFT);
    */

    if(!sim && use_camera){
        ERR_CHECK(get_hanoi_state(hanoi, tokens, sim), -3)
    }else{
        //for(int i=tokens-1; i>=0; i--){
        for(int i=7; i>=7-tokens+1; i--){
            if(!hanoi.canPlace(static_cast<hanoi_piece>(i), ROD_RIGHT)){
                ROS_ERROR("[Hanoi] Piece cannot be placed, invalid hanoi state.");
                return -1;
            }
            hanoi.addPiece(static_cast<hanoi_piece>(i), ROD_RIGHT);
        }
    }

    hanoi.printHanoi();

    //For a custom goal: define it here and pass it to solveHanoi
    /*
    Hanoi hanoiGoal;
    hanoiGoal.addPiece(TOKEN_520, ROD_RIGHT);
    hanoiGoal.addPiece(TOKEN_460, ROD_RIGHT);
    hanoiGoal.addPiece(TOKEN_375, ROD_RIGHT);
    hanoiGoal.addPiece(TOKEN_340, ROD_RIGHT);
    hanoiGoal.addPiece(TOKEN_295, ROD_RIGHT);
    hanoiGoal.addPiece(TOKEN_240, ROD_RIGHT);
    hanoiGoal.addPiece(TOKEN_200, ROD_RIGHT);
    std::vector<hanoi_move> solutionMoves;
    hanoi.solveHanoi(hanoiGoal, solutionMoves);
    */

    clock_t start = std::clock();
    std::vector<hanoi_move> solutionMoves;
    hanoi.solveHanoi(solutionMoves);
    double time = double(std::clock() - start)/CLOCKS_PER_SEC;
    std::cout << "Hanoi solver took " << time << " secs for " << tokens << " pieces!" << std::endl;
    std::cout << solutionMoves.size() << " steps to solve hanoi." << std::endl;
    //hanoi.solveHanoi(solutionMoves, hanoiGoal);
    //hanoi.printMoves(solutionMoves, true);
    // ----- HANOI SETUP END


    //ask if we should proceed (safety check whether hanoi state was detected correctly)
    std::cout << "Do you want to continue with the detected hanoi state? [y/n] ";
    char ans;
    std::cin >> ans;
    if(ans == 'n' or ans == 'N'){
        std::cout << "Exiting." << std::endl;
        return 0;
    }
    std::cout << "Continuing." << std::endl;
    //ros::Duration(10).sleep();


    //move_to_vertical();

    // Three transformations are needed to play: left, right and middle rods
    while ( not game_tfs() ) {
        ROS_INFO("Waiting for transforms from the game.");
        ros::Duration(1).sleep();
    }

    return solve_hanoi(solutionMoves);

}
