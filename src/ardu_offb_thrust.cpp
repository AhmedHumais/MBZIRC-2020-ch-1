

#include <ros/ros.h>
#include <angles/angles.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include "pid.h"

mavros_msgs::State current_state;
geometry_msgs::Point curr_pos, target_pos;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pos = (*msg).pose.position;
}

void target_sub_cb(const geometry_msgs::PointStamped::ConstPtr& msg){
    target_pos = (*msg).point;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_thrust_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::Subscriber local_pos = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pos_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/takeoff");
    ros::Subscriber target_body = nh.subscribe<geometry_msgs::PointStamped>
            ("target/body", 10, target_sub_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    float Fs = 4;
    ros::Rate rate(Fs);

    long t = 0;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = 2;

/*
    mavros_msgs::PositionTarget TarPos;
    TarPos.coordinate_frame= TarPos.FRAME_BODY_OFFSET_NED;
    TarPos.type_mask = 0b100111111000;

    geometry_msgs::Point pos;
    pos.x = 2;
    pos.y = 0;
    pos.z = -1;
    
    TarPos.position = pos;
    TarPos.yaw = 1.5708;

    bool first_ = true;

/*
    mavros_msgs::PositionTarget TarPos;
    TarPos.coordinate_frame= TarPos.FRAME_LOCAL_OFFSET_NED;
    TarPos.type_mask = 0b110111000111;

    target_pos.x = 0;
    target_pos.y = 2;
    target_pos.z = 3;

    geometry_msgs::Vector3 Vel;
*/
    PID pid_Vx = PID(1/Fs, 10, -10, 0.4, 0.05, 0.02);
    PID pid_Vy = PID(1/Fs, 10, -10, 0.4, 0.05, 0.02);
    PID pid_Vz = PID(1/Fs, 7, -7, 0.4, 0.05, 0.02);
    PID yaw_rt = PID(1/Fs, 3, -3, 0.4, 0.01, 0.01);

    mavros_msgs::PositionTarget TarPos;
    TarPos.coordinate_frame= TarPos.FRAME_BODY_OFFSET_NED;
    TarPos.type_mask = 0b010111000111;

    geometry_msgs::Vector3 Vel;

    ros::Time last_request = ros::Time::now();
    bool takeoff_comp = false;

    while(ros::ok()){
        if( current_state.mode != "GUIDED" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    if( takeoff_client.call(takeoff_cmd) && 
                        takeoff_cmd.response.success){
                            ROS_INFO("Taking Off");
                        }
                }
                last_request = ros::Time::now();
            }
        }

/*	thr.thrust = 0.8;

        local_pos_pub.publish(thr);
	ROS_INFO("Published target pose");
*/      if (!takeoff_comp){
            if (curr_pos.z > 1.95){
                takeoff_comp = true;
            }
        }
        else{
            float x = target_pos.x;
            float y = target_pos.y;
            float y_ang = atan2(target_pos.y, target_pos.x);
            if (abs(x) + abs(y) < 2){
                TarPos.yaw_rate = 0.0;
                yaw_rt.reset();
                Vel.x = pid_Vx.calculate2(x);
                Vel.y = pid_Vy.calculate2(y);
            }
            else if ( abs(angles::to_degrees(y_ang) - 90) < 5 ){
//                Vel.x = pid_Vx.calculate2(x);
                Vel.y = pid_Vy.calculate2(y);
                
                               
                TarPos.yaw_rate = yaw_rt.calculate(y_ang, angles::from_degrees(90));
            }
            else{
                TarPos.yaw_rate = yaw_rt.calculate(y_ang, angles::from_degrees(90));
                
                Vel.x = 0;
                Vel.y = 0;
                Vel.z = 0;
            }
        Vel.z = pid_Vz.calculate2(target_pos.z);
        TarPos.velocity = Vel;
        local_pos_pub.publish(TarPos);
        ROS_INFO("Published velocity");
        }
        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}
