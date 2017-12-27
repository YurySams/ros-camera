/*
 * ros_camera.h
 *
 *  Created on: Dec 5, 2017
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/camera_publisher.h>
#include <sensor_msgs/fill_image.h>

#include "base_camera.h"
#include "wqueue.h"
#include "imgframe.h"
#include "commbase.h"

namespace ros_camera {

/**
 * @brief ROS camera.
 *
 */
class Controller {
public:
    const int G_HEIGHT = 480;
    const int G_WIDTH = 640;
    const int IMAGE_SIZE = G_WIDTH * G_HEIGHT;

public:
    Controller(ros::NodeHandle& private_node, ros::NodeHandle& camera_node);

    ~Controller();

    /**
     * @brief Setup camera device and ROS parameters.
     *
     * @throw ros_camera::DeviceError device open failed.
     */
    int setup();

    int run();

    static void* thread_proc_receiver(void* arg);
    static void* thread_proc_callback(void* arg);
    void proc_receiver();
    void proc_callback();
    void acync_receiver();
    void image_ready();

protected:
    // Callback function when data come from camera.
    virtual void OnDeviceData(const imgframe& img);

private:
    void publish_topic(const imgframe& img);
    uint64_t stamp_;

private:
    BaseCamera device;

    /**
     * @brief ROS private node for getting ROS parameters.
     */
    ros::NodeHandle private_node_;

    /**
     * @brief ROS private node for publishing images.
     */
    ros::NodeHandle camera_node_;

    // TODO: redesign!!!
    wqueue<imgframe*> *disqueue;
    Thread thread_receiver;
    Thread thread_event;
    byte* m_pOutData; //send data buffer
    bool f_capture;
    std::string camera_name_, camera_info_url_;

    // TODO: why???
    sensor_msgs::Image img_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
    image_transport::CameraPublisher camera_pub;

    bool thread_receiver_ready, thread_event_ready;

};

} // namespace ros_camera
