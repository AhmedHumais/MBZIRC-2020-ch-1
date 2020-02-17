

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PointStamped>
            ("target/body", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    long t = 0;

    // wait for FCU connection

    geometry_msgs::PointStamped pose, point_out;
    pose.point.x = 5;
    pose.point.y = 10;
    pose.point.z = 2.5;


    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        try {
            tfBuffer.transform(pose, point_out, "body", ros::Duration(0.4));
            local_pos_pub.publish(point_out);
    	    ROS_INFO("Published target pose");       
        }
        catch (tf2::TransformException &ex) {
           ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
