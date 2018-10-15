#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sstream>

using namespace std; 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  
  ros::init(argc, argv, "navigation_simple");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient move_base("move_base", true);

  //wait for the action server to come up
  while(!move_base.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  double X ;
  double Y ;

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("X", X, double(4));
  private_node_handle_.param("Y", Y, double(4));

  move_base_msgs::MoveBaseGoal goal;

  //move turtlebot to specific position in relation to the turtlebot
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = X;
  goal.target_pose.pose.position.y = Y;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  move_base.sendGoal(goal);

  move_base.waitForResult();

  stringstream message;
  message << "Hooray, the base moved " << X << " meters in the X direction and " << Y << " in the Y direction";  

  if(move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM(message.str());
  else
    ROS_INFO("The base failed to move that location for some reason");

  return 0;
}