#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <fstream>


// TODO: Check that this file routing will always work for any catkin_ws
const char store_board_directory[] = "stored_board_pose.dat";
const char store_joint_directory[] = "stored_joints.dat";


struct Joints{
    std::string name;
    double j1, j2, j3, j4, j5, j6;
};


struct BoardPose{
    float x, y, z, qx, qy, qz, qw;
};


void joints_2_file(const std::vector<Joints> js){
    std::ofstream outFile;
    outFile.open(store_joint_directory);
    for (int i{0}; i < js.size(); i+=1){
        outFile << js[i].name << "\t" << js[i].j1 << "\t" << js[i].j2 << "\t" << js[i].j3 << "\t" << js[i].j4 << "\t" << js[i].j5 << "\t" << js[i].j6 << std::endl;
    }
    outFile.close();
}


auto get_stored_joint_vals(){
    std::ifstream inFile;
    inFile.open(store_joint_directory);
    std::vector<Joints> read_joints;
    if (inFile.fail()){
        ROS_WARN("Could not find file 'stored_joints.dat' with previously stored values.\nWill measure instead.\n");
        return read_joints;
    }
    else {
        std::string name;
        float j1, j2, j3, j4, j5, j6;
        while (inFile >> name >> j1 >> j2 >> j3 >> j4 >> j5 >> j6){
            read_joints.push_back(Joints{name, j1, j2, j3, j4, j5, j6});
        }
        return read_joints;
    }
}


auto get_stored_board_tf(){
    std::ifstream inFile;
    inFile.open(store_board_directory);
    BoardPose board{0,0,0,0,0,0,0};

    if (inFile.fail()){
        ROS_WARN("Could not find file 'stored_board_pose.dat' with previously stored values. Will measure instead.\n");
        inFile.close();
        return board;
    }
    else {
        ROS_INFO("Reading board coordinates from file.");
        inFile >> board.x >> board.y >> board.z >> board.qx >> board.qy >> board.qz >> board.qw;
        std::cout << board.x << "\t"<< board.y << "\t"<< board.z << "\t"<< board.qx << "\t"<< board.qy << "\t"<< board.qz << "\t"<< board.qw << std::endl;
        inFile.close();
        return board;
    }
}


auto get_current_board_tf(){
    tf::TransformListener listener;
    tf::StampedTransform board_tf;
    auto now = ros::Time::now();
    listener.waitForTransform("/edo/edo_base_link", "/board_connect_4_base_link", now, ros::Duration(3.));
    listener.lookupTransform("/edo/edo_base_link", "/board_connect_4_base_link", now, board_tf);
    return board_tf;
}


void board_2_file(){
    auto board = get_current_board_tf();
    std::ofstream outFile;
    outFile.open(store_board_directory, std::fstream::out);
    outFile << board.getOrigin().x() << "\t" << board.getOrigin().y() << "\t" << board.getOrigin().z() << "\t" << board.getRotation().x() << "\t" << board.getRotation().y() << "\t" << board.getRotation().z() << "\t" << board.getRotation().w();
    outFile.close();
}


bool are_floats_same(float a, float b){
    float epsilon = 0.001;
    return std::abs(1 - b) < epsilon; //TODO:MAke sure you replace 1 with a later when done.
}


bool board_moved(){
// Checks if the board moved with respect to last time

    auto prev_board_tf = get_stored_board_tf();

    auto board_tf = get_current_board_tf(); //note: should make a class and overload the comparison operator

    auto mb_x = (float)(board_tf.getOrigin().x());
    auto mb_y = (float)(board_tf.getOrigin().y());
    auto mb_z = (float)(board_tf.getOrigin().z());
    auto mb_qx = (float)board_tf.getRotation().x();
    auto mb_qy = (float)board_tf.getRotation().y();
    auto mb_qz = (float)board_tf.getRotation().z();
    auto mb_qw = (float)board_tf.getRotation().w();


    //TODO:Improve this to compare floats using epsilon
    if (are_floats_same(prev_board_tf.x, mb_x) and are_floats_same(prev_board_tf.y, mb_y) and are_floats_same(prev_board_tf.z, mb_z) and are_floats_same(prev_board_tf.qx, mb_qx) and are_floats_same(prev_board_tf.qy, mb_qy) and are_floats_same(prev_board_tf.qz, mb_qz) and are_floats_same(prev_board_tf.qw, mb_qw)){
        std::cout << "Board is in the same tf as last time." << std::endl;
        return false;
    }

    ROS_INFO("The board moved.");
    return true;
}
