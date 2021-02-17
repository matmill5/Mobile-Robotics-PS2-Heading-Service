//example ROS client:
// first run: rosrun heading_ros_service heading_ROS_service
// then start this node:  rosrun heading_ros_service heading_ros_client



#include <ros/ros.h>
#include <heading_ros_service/ExampleServiceMsg.h> // this message type is defined in the current package
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_ros_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<heading_ros_service::ExampleServiceMsg>("heading_controller_service");
    heading_ros_service::ExampleServiceMsg srv;
    double desired_heading;
    while (ros::ok()) {
        cout << "Enter a desired heading: ";
    	cin  >> desired_heading;
    	srv.request.desired_heading = desired_heading;
   	client.call(srv);
    }
 
    return 0;
}
