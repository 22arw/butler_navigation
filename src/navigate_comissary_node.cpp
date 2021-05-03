#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

bool moveToPosition(double xPosition, double yPosition);

// coordinates of locations to navigate to
double xPositions[] = {29.14};
double yPositions[] = {22.59};

int sizeOfArray = sizeof(xPositions)/sizeof(xPositions[0]);

bool positionReached = false;
int main(int argc, char** argv) {
    ros::init(argc, argv, "navigate_comissary_node");
    ros::NodeHandle n;
    ros::spinOnce();
    
    bool goalSuccess = true;

    while (goalSuccess) {
        for (int i=1; i<sizeOfArray; i++) {
            positionReached = moveToPosition(xPositions[i], yPositions[i]);
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

bool moveToPosition(double xPosition, double yPosition) {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("move_base action server has not come up yet");
    }

    move_base_msgs::MoveBaseGoal position;

    position.target_pose.header.frame_id = "map";
    position.target_pose.header.stamp = ros::Time::now();


    position.target_pose.pose.position.x = xPosition;
    position.target_pose.pose.position.y = yPosition;
    position.target_pose.pose.position.z = 0.0;
    position.target_pose.pose.orientation.x = 0.0;
    position.target_pose.pose.orientation.y = 0.0;
    position.target_pose.pose.orientation.z = 0.0;
    position.target_pose.pose.orientation.w = 0.0;

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