#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

bool moveToPosition(double xPosition, double yPosition, double wOrientation);

// coordinates of locations and pose to navigate to
double xPositions[] = {29.14, 25.73, 25.76, 21.63, 21.77, 32.74, 32.74, 29.27};
double yPositions[] = {22.59, 22.87, 39.4, 38.91, 22.88, 22.73, 38.68, 38.53};
double wOrientations[] = {0.05, 0.69, 0.009, 0.71, 0.99, 0.71, 0.01, 0.71};

int sizeOfArray = sizeof(xPositions)/sizeof(xPositions[0]);

bool positionReached = false;
int main(int argc, char** argv) {
    ros::init(argc, argv, "navigate_comissary_node");
    ros::NodeHandle n;
    ros::spinOnce();
    
    bool goalSuccess = true;

    ROS_INFO("Starting robot journey");
    while (goalSuccess) {
        for (int i=0; i<sizeOfArray; i++) {
            ROS_INFO("sending command");
            positionReached = moveToPosition(xPositions[i], yPositions[i], wOrientations[i]);
            if (positionReached) {
                ROS_INFO("Reached position, running next position");
                goalSuccess = true;
            }
            else {
                ROS_INFO("Error reaching goal breaking out of program");
                goalSuccess = false;
            }
        }
    }
}

bool moveToPosition(double xPosition, double yPosition, double wOrientation) {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("move_base action server has not come up yet");
    }

    move_base_msgs::MoveBaseGoal position;

    position.target_pose.header.frame_id = "map";
    position.target_pose.header.stamp = ros::Time::now();


    position.target_pose.pose.position.x = xPosition;
    position.target_pose.pose.position.y = yPosition;
    position.target_pose.pose.orientation.w = wOrientation;


    ROS_INFO("Sending position coordinates to move_base action server");
    ac.sendGoal(position);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        return true;
    }
    else {
        return false;
    }
    
}