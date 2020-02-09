#include <ros/ros.h>
#include "zed_depth.hpp"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <signal.h>


void intrpt_handler(int signum) {
    ROS_INFO("Interrupted, exiting");
    exit_flag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_node");
    ros::NodeHandle nh;

    std::string cfg_file = "/home/ahmed/trained-nets/balloon/balloon_tiny_30jan.cfg";
    std::string weights_file = "/home/ahmed/trained-nets/balloon/balloon_tiny_30jan_final.weights";

    ros::Publisher obj_pos_pub = nh.advertise<geometry_msgs::PointStamped>
            ("cam/objpos", 10);
    ros::Publisher bb_pub = nh.advertise<geometry_msgs::Quaternion>
            ("cam/BB", 10);

    ros::Rate rate(15.0);

    cv::VideoCapture cap(1);

    while (!cap.isOpened()){
        ROS_ERROR("Camera not connected");
        ros::spinOnce();
        exit(1);
    }
    signal(SIGINT, intrpt_handler);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    float const thresh = 0.25;
//    auto obj_names = objects_names_from_file(names_file);
    geometry_msgs::PointStamped ObjPos;
    geometry_msgs::Quaternion bb;

    cap >> cur_frame;
    exit_flag = false;
    new_data = false;

    std::thread detect_thread(detectorThread, cfg_file, weights_file, thresh);
    sl::sleep_ms(1000);
    while (ros::ok() && (!exit_flag)) {
        cap >> cur_frame;

        if (!cur_frame.empty()) {
            data_lock.lock();
            new_data = true;
            data_lock.unlock();

            while(new_data){
                ros::spinOnce();
            }
            if (!result_vect.empty()){
                data_lock.lock();
                auto Bbox = get_largest_bb(result_vect);
                data_lock.unlock();
                ObjPos.header.stamp = ros::Time::now();
                ObjPos.point.x = Bbox.x + Bbox.w*0.5 - 640.0;
                ObjPos.point.z = 360.0 - (Bbox.y + Bbox.h*0.5);
                ObjPos.point.y = 180.0/(sqrt(Bbox.w*Bbox.h));
                obj_pos_pub.publish(ObjPos);
                
                bb.x = Bbox.x;
                bb.y = Bbox.y;
                bb.z = Bbox.h;
                bb.w = Bbox.w;
                bb_pub.publish(bb);
                ROS_INFO("Object detected");
            }
            else{
                // ObjPos.x = 0;
                // ObjPos.z = 0;
                // ObjPos.y = 0;
                // obj_pos_pub.publish(ObjPos);
                ROS_INFO("Object not detected");
            }
        }
        else{
            exit_flag = true;
        }
        ros::spinOnce();
        rate.sleep();

    }

    detect_thread.join();
    cap.release();
    return 0;
}