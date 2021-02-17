//example ROS service:
// run this as: rosrun example_ROS_service example_ROS_service
// in another window, tickle it manually with (e.g.): 
//    rosservice call lookup_by_name 'Ted'

// heading service: recieves requests for desired heading of stdr robot, 
// makes the corresponding adjustments, and returns a float64 indicating 
// status (1-success, 0-failure)

#include <ros/ros.h>
#include <heading_ros_service/ExampleServiceMsg.h>
#include <iostream>
#include <string>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

using namespace std;

ros::Publisher *g_twist_commander_ptr;
tf::TransformListener *g_tf_listener_ptr;
tf::StampedTransform g_robot_wrt_world_stf;
double Kp = 5; //2.33;
double HEADING_TOL = 0.002;
double dt = 0.01;

double heading_from_tf(tf::StampedTransform stf){
    tf::Quaternion quat;
    quat = stf.getRotation();
    double theta = tf::getYaw(quat);
    return theta;
}

bool callback(heading_ros_service::ExampleServiceMsgRequest& request, heading_ros_service::ExampleServiceMsgResponse& response)
{
    ROS_INFO("my_heading_service callback activated");
    double desired_heading = request.desired_heading;
    double actual_heading;
    
    
    // Perform action to carry out request
    // Logic goes here
    
    geometry_msgs::Twist twist_cmd;
    
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;
    
    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;
    
    g_tf_listener_ptr->lookupTransform("world", "robot0", ros::Time(0), g_robot_wrt_world_stf);
    
    actual_heading = heading_from_tf(g_robot_wrt_world_stf);
    desired_heading = std::fmod(actual_heading + desired_heading,(2*M_PI));
    // ROS_INFO("received request to rotate to heading of %f", desired_heading);

    //Feedback, heading logic
    double heading_err = 100;

    while(fabs(heading_err)>HEADING_TOL){
    	g_tf_listener_ptr->lookupTransform("world", "robot0", ros::Time(0), g_robot_wrt_world_stf);
    	
    	//Extract heading
    	actual_heading = heading_from_tf(g_robot_wrt_world_stf);
    	ROS_INFO("current heading is %f", actual_heading);
    	
      //error
      heading_err = desired_heading - actual_heading;
      if(heading_err>M_PI) heading_err-=2*M_PI;
    	if(heading_err<-M_PI) heading_err+=2*M_PI;
      
    	ROS_INFO("heading error = %f", heading_err);
    	
    	twist_cmd.angular.z = Kp*heading_err;
      g_twist_commander_ptr->publish(twist_cmd);
    	ros::spinOnce();
    	ros::Duration(dt).sleep();
    }
    
    // Make the robot stop completely after finish the command
    response.heading_achieved = true;
    for (int i=0; i<10; i++){
      twist_cmd.angular.z = 0;
      g_twist_commander_ptr->publish(twist_cmd);
      ros::spinOnce();
    }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "heading_ros_service");
  ros::NodeHandle n;

  ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
  g_twist_commander_ptr = &twist_commander;
  ros::ServiceServer service = n.advertiseService("heading_controller_service", callback);
  
  //Retrieve odometry information from the robot
  // while loop to warm up tf-listener
  g_tf_listener_ptr = new tf::TransformListener;
  
  ROS_INFO("Trying to get robot pose w/rt to world");
  bool tferr = true;
  while(tferr){
  	tferr = false;
  	try {
  	   g_tf_listener_ptr->lookupTransform("world", "robot0", ros::Time(0), g_robot_wrt_world_stf);
  	} catch(tf::TransformException &exception){
  		ROS_WARN("%s; retrying...", exception.what());
  		tferr = true;
  		ros::spinOnce();
  	}
  }
  
  ROS_INFO("Ready to control robot's orientation");
  ros::spin();

  return 0;
}
