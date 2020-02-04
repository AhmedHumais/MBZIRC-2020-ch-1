#include <ros/ros.h>
#include "zed_depth.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <signal.h>


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

    ros::Publisher obj_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("zed/objpos", 10);
    ros::Rate rate(15.0);

    sl::Camera zed;
    sl::InitParameters init_params;
    sl::SensorsData data;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
 //   init_params.camera_fps = 15;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.depth_minimum_distance = 0.2;
    init_params.depth_maximum_distance = 20;
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

    float const thresh = 0.25;
//    auto obj_names = objects_names_from_file(names_file);

    sl::Mat left, cur_cloud;
    sl::SensorsData sen_data;
    sl::float3 obj_coord;
    geometry_msgs::PoseStamped ObjPos;
    ObjPos.header.frame_id = "camera";

    zed.retrieveImage(left);
    zed.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);
    slMat2cvMat(left).copyTo(cur_frame);
    exit_flag = false;
    new_data = false;

    std::thread detect_thread(detectorThread, cfg_file, weights_file, thresh);
    sl::sleep_ms(1000);
    while (ros::ok() && (!exit_flag)) {

        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(left);
            if( zed.getSensorsData( data, sl::TIME_REFERENCE::IMAGE ) != sl::ERROR_CODE::SUCCESS ) {
                ROS_WARN("Sensor data not available");
            }
            data_lock.lock();
            cur_frame = slMat2cvMat(left);
            new_data = true;
            data_lock.unlock();

            zed.retrieveMeasure(cur_cloud, sl::MEASURE::XYZ);

            data_lock.lock();
            auto result_vec_draw = getObjectDepth(result_vect, cur_cloud);
            auto objs_coord = get_3d_coords(result_vec_draw);
            data_lock.unlock();
            if(!objs_coord.empty()){
                obj_coord = objs_coord.front();
                auto cam_orient = data.imu.pose.getOrientation();
                ObjPos.header.stamp = ros::Time::now();
                ObjPos.pose.position.x = obj_coord.x;
                ObjPos.pose.position.y = obj_coord.y;
                ObjPos.pose.position.z = obj_coord.z;
                ObjPos.pose.orientation.w = cam_orient.ow;
                ObjPos.pose.orientation.x = cam_orient.ox;
                ObjPos.pose.orientation.y = cam_orient.oy;
                ObjPos.pose.orientation.z = cam_orient.oz;

                obj_pos_pub.publish(ObjPos);
                ROS_INFO("Object detected");
            }
            else{
                ROS_INFO("Object not detected");
            }

        }
        ros::spinOnce();
        rate.sleep();

    }

    detect_thread.join();
    zed.close();
    return 0;
}