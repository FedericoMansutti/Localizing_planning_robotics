#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "navigation_goals");
    ros::NodeHandle n;
    
    /*
    std::ifstream file;
    
    file.open("../waypoints.csv");
    if (!file.is_open()){
   	ROS_INFO("error opening file");
  	ROS_INFO("error opening file");
   	ROS_INFO("error opening file");
    	return 1;
    }    // read file path from parameter file
    
    */
    std::string path;
    if (argc == 2) {
        path = argv[1];
    } else {
        ROS_INFO("Error: no path specified");
        return 1;
    }

    // read waypoints.csv file whith c++ fstream and for each line copy the fist value in x array, the second in y array and the third in heading array
    std::ifstream file(path, std::ifstream::in);


    ROS_INFO("Reading waypoints.csv file");
    std::vector<double> x_values, y_values, orientation;
    std::string line;

    while (std::getline(file, line)) { // Read each line from the CSV file
        std::stringstream ss(line); // Create a stringstream from the line

        double value1, value2, value3;
        char comma; // To read and discard the commas
        if (ss >> value1 >> comma >> value2 >> comma >> value3) {
            x_values.push_back(value1);
            y_values.push_back(value2);
            orientation.push_back(value3);
            ROS_INFO("%f", value1);
            


        }
    }

    ROS_INFO("Copying values in x, y and heading array");
    // create 3 array x, y and heading and copy the values from the vector values, the first value of each row in x array, the second in y array and the third in heading array
    int length = x_values.size();
    double x [length];
    double y [length];
    double heading [length];
    for (int i = 0; i < length; i++) {
        x[i] = x_values[i];
        y[i] = y_values[i];
        heading[i] = orientation[i];
    }

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    for( int i = 0; i < length; i = i + 1 ) {
        goal.target_pose.pose.position.x = x[i];
        goal.target_pose.pose.position.y = y[i];
        goal.target_pose.pose.orientation.z = heading[i];
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal %d", i);
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Goal reached");
        else
            ROS_INFO("The robot failed to reach the goal for some reason");
    }

    return 0;
}