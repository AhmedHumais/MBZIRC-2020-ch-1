#include <ros/ros.h>
#include "zed_depth.hpp"
#include <geometry_msgs/PointStamped.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <thread>

void trackerThread(const ros::Publisher &pub) {
    cv::Ptr<cv::Tracker> tracker;
    bbox_t BB_object;
    cv::Rect2d BB_track2d;
    bool ok = false;
    geometry_msgs::PointStamped target_pos;
    target_pos.header.frame_id = "camera";
    cv::Mat track_frame;
    ros::Time last_detection; 

    while(!exit_flag){
        if(start_tracking){
            frame_lock.lock();
            cv::cvtColor(frame, track_frame, CV_RGBA2RGB);
            frame_lock.unlock();
            if (new_detect){
                last_detection = ros::Time::now();
                bb_lock.lock();
                BB_track2d.x = det_bb.x;  
                BB_track2d.y = det_bb.y;  
                BB_track2d.height = det_bb.h;  
                BB_track2d.width = det_bb.w;  
                bb_lock.unlock();
//                tracker->clear();
                tracker = cv::TrackerCSRT::create();
                if(!(tracker->init(track_frame, BB_track2d))){
                    ROS_INFO("Tracker init failed");
                }
                new_detect = false;

                ok = true;
            }
            else{
                if (ros::Time::now() - last_detection > ros::Duration(2)){
                    ok = false;
                }
                else{
                    ok = tracker->update(track_frame, BB_track2d);
                }
            }

            if (ok){
                BB_object.x = BB_track2d.x;
                BB_object.y = BB_track2d.y;
                BB_object.h = BB_track2d.height;
                BB_object.w = BB_track2d.width;
                while(!new_cloud){
                    sl::sleep_ms(1);
                }
                cloud_lock.lock();
                auto BB_object3d = get_3d_BB(BB_object, cur_cloud);
                cloud_lock.unlock();
                new_cloud = false;
                target_pos.header.stamp = ros::Time::now();
                target_pos.point.x = BB_object3d.coord.x;
                target_pos.point.y = BB_object3d.coord.y - 0.5;
                target_pos.point.z = BB_object3d.coord.z;
                pub.publish(target_pos);
                ROS_INFO("published position");

                cv::rectangle(track_frame, BB_track2d, cv::Scalar(255, 0, 0), 2, 1);
                cv::imshow("Tracking", track_frame);
                cv::waitKey(1);
            }
            else{
                new_frame = true;
                sl::sleep_ms(10);

                ROS_INFO("Lost Tracking");
                start_tracking = false;
            }
        }
        else{
            new_frame = true;
            sl::sleep_ms(10);
            ROS_INFO("Nothing to track");
        }
        sl::sleep_ms(5);
    }
}

void intrpt_handler(int signum) {
    ROS_INFO("Interrupted, exiting");
    exit_flag = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_objdetect_node");
    ros::NodeHandle nh;

    std::string cfg_file = "/home/ahmed/trained-nets/balloon/balloon_tiny_30jan.cfg";
    std::string weights_file = "/home/ahmed/trained-nets/balloon/balloon_tiny_30jan_final.weights";

    ros::Publisher obj_pos_pub = nh.advertise<geometry_msgs::PointStamped>
            ("zed/objpos", 10);

    sl::Camera zed;
    sl::InitParameters init_params;
//    sl::SensorsData data;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_minimum_distance = 0.15;
    init_params.depth_maximum_distance = 20;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP;

    auto err = zed.open(init_params);
    while (err != sl::ERROR_CODE::SUCCESS){
        ROS_INFO("Can't connect to camera");
        ROS_ERROR("Camera not connected");
        ros::spinOnce();
        err = zed.open(init_params);
        exit(1);
    }
    signal(SIGINT, intrpt_handler);
    zed.grab();

    float const thresh = 0.75;
//    auto obj_names = objects_names_from_file(names_file);

    sl::Mat left;
    // sl::SensorsData sen_data;
 //   sl::float3 obj_coord;
    geometry_msgs::PointStamped ObjPos;
    ObjPos.header.frame_id = "camera";

    zed.retrieveImage(left);
    zed.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);
    slMat2cvMat(left).copyTo(cur_frame);
    exit_flag = false;
    new_data = false;

    std::thread detect_thread(detectorThread, cfg_file, weights_file, thresh);
    sl::sleep_ms(1000);
   int frame_count = 0;

    std::thread tracker_thread(trackerThread, obj_pos_pub);
    while (ros::ok() && (!exit_flag)) {
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(left);

            frame_lock.lock();
            frame = slMat2cvMat(left);
            frame_lock.unlock();

            if(frame_count > 5){
                new_frame = true;
                frame_count = 0;
                ROS_INFO("Detecting");
            }
            else{
                frame_count +=1;
            }

            cloud_lock.lock();
            zed.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);
            cloud_lock.unlock();
            new_cloud = true;

        }
        else{
            ROS_ERROR("frame_not_available");
        }
    }

    tracker_thread.join();
    detect_thread.join();
    zed.close();
    return 0;
}