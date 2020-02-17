#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <ros/ros.h>

// This is a modified version of https://github.com/AlexeyAB/darknet/blob/master/src/yolo_console_dll.cpp
// Basically simplified and using the ZED SDK

#define OPENCV
#define GPU

#include <sl/Camera.hpp>

#include "yolo_v2_class.hpp"    // https://github.com/AlexeyAB/darknet/blob/master/src/yolo_v2_class.hpp
#include <opencv2/opencv.hpp>

class bbox_t_3d {
public:
    bbox_t bbox;
    sl::float3 coord;

    bbox_t_3d(bbox_t bbox_, sl::float3 coord_) {
        bbox = bbox_;
        coord = coord_;
    }
};

float getMedian(std::vector<float> &v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

std::vector<bbox_t_3d> getObjectDepth(std::vector<bbox_t> &bbox_vect, sl::Mat &xyzrgba) {
    sl::float4 out(NAN, NAN, NAN, NAN);
    bool valid_measure;
    int i, j;
    const int R_max = 4;

    std::vector<bbox_t_3d> bbox3d_vect;

    for (auto &it : bbox_vect) {

        int center_i = it.x + it.w * 0.5f, center_j = it.y + it.h * 0.5f;

        std::vector<float> x_vect, y_vect, z_vect;
        for (int R = 0; R < R_max; R++) {
            for (int y = -R; y <= R; y++) {
                for (int x = -R; x <= R; x++) {
                    i = center_i + x;
                    j = center_j + y;
                    xyzrgba.getValue<sl::float4>(i, j, &out, sl::MEM::CPU);
                    valid_measure = std::isfinite(out.z);
                    if (valid_measure) {
                        x_vect.push_back(out.x);
                        y_vect.push_back(out.y);
                        z_vect.push_back(out.z);
                    }
                }
            }
        }

        if (x_vect.size() * y_vect.size() * z_vect.size() > 0) {
            float x_med = getMedian(x_vect);
            float y_med = getMedian(y_vect);
            float z_med = getMedian(z_vect);

            bbox3d_vect.emplace_back(it, sl::float3(x_med, y_med, z_med));
        }
    }

    return bbox3d_vect;
}

bbox_t_3d get_3d_BB(bbox_t bbox, sl::Mat &xyzrgba) {
    sl::float4 out(NAN, NAN, NAN, NAN);
    bool valid_measure;
    int i, j;
    const int R_max = 4;
    bbox_t_3d bbox3d(bbox, sl::float3(0, 0, 0));

    int center_i = bbox.x + bbox.w * 0.5f, center_j = bbox.y + bbox.h * 0.5f;

    std::vector<float> x_vect, y_vect, z_vect;
    for (int R = 0; R < R_max; R++) {
        for (int y = -R; y <= R; y++) {
            for (int x = -R; x <= R; x++) {
                i = center_i + x;
                j = center_j + y;
                xyzrgba.getValue<sl::float4>(i, j, &out, sl::MEM::CPU);
                valid_measure = std::isfinite(out.z);
                if (valid_measure) {
                    x_vect.push_back(out.x);
                    y_vect.push_back(out.y);
                    z_vect.push_back(out.z);
                }
            }
        }
    }

    if (x_vect.size() * y_vect.size() * z_vect.size() > 0) {
        float x_med = getMedian(x_vect);
        float y_med = getMedian(y_vect);
        float z_med = getMedian(z_vect);

        bbox3d = bbox_t_3d(bbox, sl::float3(x_med, y_med, z_med));
    }

    return bbox3d;
}


bbox_t get_largest_bb(std::vector<bbox_t> &bbox_vect){
    float largest;
    int i = 0;
    bbox_t BB;
    for (auto &it : bbox_vect) {

        float center_i = it.x + it.w * 0.5f, center_j = it.y + it.h * 0.5f;
        float area = it.w*it.h;
        if (i==0){
            i++;
            largest = area;
            BB = it;
        }
        else{
            if (largest < area){
                largest = area;
                BB = it;
            }
        }
    }
    return BB;
}

std::vector<sl::float3> get_3d_coords(std::vector<bbox_t_3d> result_vec){
    std::vector<sl::float3> obj_coords;
    for (auto &i : result_vec) {
        obj_coords.push_back(i.coord);
    }
    return obj_coords;
}


cv::Mat slMat2cvMat(sl::Mat &input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE::F32_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE::F32_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE::F32_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE::F32_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE::U8_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE::U8_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE::U8_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE::U8_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}


std::mutex frame_lock, bb_lock, cloud_lock, data_lock;
cv::Mat frame, cur_frame;
sl::Mat cur_cloud; 
std::vector<bbox_t> result_vect;
std::atomic<bool> exit_flag, new_frame, start_tracking, new_cloud, new_detect, new_data;
bbox_t det_bb;

void detectorThread(std::string cfg_file, std::string weights_file, float thresh) {
    Detector detector(cfg_file, weights_file);
    std::shared_ptr<image_t> det_image;
    std::vector<bbox_t> det_vect;
    cv::Size const frame_size = frame.size();

    while (!exit_flag) {
        if (new_frame) {
            frame_lock.lock();
            det_image = detector.mat_to_image_resize(frame);
            frame_lock.unlock();
            det_vect = detector.detect_resized(*det_image, frame_size.width, frame_size.height, thresh, false); // true
            new_frame = false;
            if(!det_vect.empty()){
                bb_lock.lock();
                det_bb = get_largest_bb(det_vect);
                if((det_bb.w * det_bb.h) > 150.0){
                    new_detect = true;
                    start_tracking = true;
                    ROS_INFO("Detected");
                }
                bb_lock.unlock();
//                ROS_INFO("Detected");
            }
            
        } else sl::sleep_ms(1);
    }
}

