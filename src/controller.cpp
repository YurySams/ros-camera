/*
 * ros_camera.cpp
 *
 *  Created on: Dec 5, 2017
 *
 *  There are two implementation:
 *  1. Both read and image transfer are working in one thread.
 *  2. Read and transfer are working in determinated thread.
 *  Need to evaluate difference.
 */

#include "../include/ros_camera/controller.h"

namespace ros_camera {

Controller::Controller(ros::NodeHandle &private_node, ros::NodeHandle &camera_node) :
        private_node_(private_node), camera_node_(camera_node), device(G_HEIGHT, G_WIDTH) {

    m_pOutData = new byte[IMAGE_SIZE];
    f_capture = false;
    thread_event_ready = false;
    thread_receiver_ready = false;

    image_transport::ImageTransport itp(private_node_);
    camera_pub = itp.advertiseCamera("image", 1);
    private_node_.param("camera_name", camera_name_, std::string("mt9v034"));
    private_node_.param("camera_info_url", camera_info_url_, std::string(""));
    private_node_.param("camera_frame_id", img_.header.frame_id, std::string("camera"));

    cinfo_.reset(
            new camera_info_manager::CameraInfoManager(private_node_, camera_name_,
                    camera_info_url_));
    if (!cinfo_->isCalibrated()) {
        cinfo_->setCameraName(camera_name_);
        sensor_msgs::CameraInfo camera_info;
        camera_info.header.frame_id = img_.header.frame_id;
        camera_info.width = G_WIDTH;
        camera_info.height = G_HEIGHT;
        cinfo_->setCameraInfo(camera_info);
    }

    disqueue = new wqueue<imgframe*>();
}

Controller::~Controller() {
    f_capture = false;

    disqueue->reset();

    thread_event.join();
    thread_receiver.join();

    delete disqueue;

    if (m_pOutData != NULL) {
        delete (m_pOutData);
        m_pOutData = NULL;
    }
    std::cout << "ROS camera destroyed...." << std::endl;
}

int Controller::setup() {
    return device.open();
}

int Controller::run() {
    ROS_DEBUG_STREAM("Camera::run. Enter.");

    thread_event_ready = false;
    if (thread_event.start(thread_proc_callback, (void*) this) == false)
        return -1;
    while (thread_event_ready == false) {
        msleep(10);
    }

    thread_receiver_ready = false;
    if (thread_receiver.start(thread_proc_receiver, (void*) this, 160 * 1024) == false) {
        return -1;
    }
    while (thread_receiver_ready == false) {
        msleep(10);
    }

    ROS_INFO_STREAM("Camera::run. Exit.");
    return 0;
}

void* Controller::thread_proc_callback(void* arg) {
    ((Controller*) arg)->proc_callback();
    return NULL;
}

void* Controller::thread_proc_receiver(void* arg) {
    pthread_t thId = pthread_self();
    pthread_attr_t thAttr;
    int policy = 0;
    int max_prio_for_policy = 0;
    pthread_attr_init(&thAttr);
    pthread_attr_getschedpolicy(&thAttr, &policy);
    max_prio_for_policy = sched_get_priority_max(policy);
    pthread_setschedprio(thId, 90);
    pthread_attr_destroy(&thAttr);
    ROS_INFO_STREAM("Thread priority=" << max_prio_for_policy << " " << sched_get_priority_min(policy));


    ((Controller*) arg)->proc_receiver();
    return NULL;
}

void Controller::proc_receiver(void) {
    ROS_INFO_STREAM("Camera. proc_receiver. Enter12.");
    thread_receiver_ready = true;

    f_capture = true;
    // Run device
    device.run();

    while (f_capture) {
        device.GetImage(m_pOutData, IMAGE_SIZE);

        // Below code block to calculate FPS.
/*
        uint64_t tmp = stamp_;
        stamp_ = ros::Time::now().toNSec();
        ROS_INFO_STREAM(
                "FPS=" << 1000000000 / (stamp_ - tmp) << "; stamp_=" << stamp_ << "; Period=" << (stamp_ - tmp));
*/
        // Send image to transfer thread.
        // TODO: optimize here by removing memory copy.
        imgframe* inputframe = new imgframe(G_WIDTH, G_HEIGHT, 0);
        std::memcpy(inputframe->imgBuf, m_pOutData, IMAGE_SIZE);
        disqueue->add(inputframe);

        /*
         // Publish
         img_.header.stamp = ros::Time::now();
         sensor_msgs::fillImage(img_, "mono8", G_HEIGHT, G_WIDTH, G_WIDTH, m_pOutData);

         sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
         ci->header.frame_id = img_.header.frame_id;
         ci->header.stamp = img_.header.stamp;
         camera_pub.publish(img_, *ci);
         */
    }
    device.stop();
    ROS_INFO_STREAM("Camera. proc_receiver. Exit.");
}

void Controller::proc_callback() {
    ROS_INFO_STREAM("Camera. proc_callback enter....");

    this->thread_event_ready = true;

    imgframe* item;
    while (!disqueue->stop) {
        item = disqueue->remove(); //will loop to get item until success
        if (item != NULL)
            OnDeviceData(*item);
        delete item;
    }
    std::cout << "proc_callback exit...." << std::endl;
}

void Controller::OnDeviceData(const imgframe& img) {
    publish_topic(img);
}

void Controller::publish_topic(const imgframe& img) {

    sensor_msgs::fillImage(img_, "mono8", img.m_heigth, img.m_width, img.m_width, img.imgBuf);
    // TODO: !!!!Move it!!!!!
    img_.header.stamp = ros::Time::now();

    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
//    ci->header.stamp = img_.header.stamp;
    ci->header.stamp = img.m_timestamp;

    camera_pub.publish(img_, *ci);
}

} //namespace ros_camera

