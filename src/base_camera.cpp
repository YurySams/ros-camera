/*
 * device.cpp
 *
 *  Created on: Dec 5, 2017
 *
 */

#include "../include/ros_camera/base_camera.h"

#include <iostream>
#include <ros/ros.h>

namespace ros_camera {

BaseCamera::BaseCamera(int height, int width) :
        devh_(NULL), height_(height), width_(width) {
}

BaseCamera::~BaseCamera() {
    close();
}

int BaseCamera::open() {
    int result = 0;
    int r;

    ROS_INFO_STREAM("device::open. Enter.");
    r = libusb_init(NULL);
    if (libusb_error::LIBUSB_SUCCESS != r) {
        // TODO: handle correctly error.
        ROS_ERROR_STREAM("Cannot init usblib. error=" << r << std::endl);
        return r;
    }

    // TODO: to find place for 0x52CB and 0x52CB.
//    devh_ = libusb_open_device_with_vid_pid(NULL, 0x52CB, 0x52CB);
    devh_ = libusb_open_device_with_vid_pid(NULL, 0xAE86, 0x0001);

    if (NULL == devh_) {
        // TODO: handle correctly error.
        ROS_ERROR_STREAM("Cannot find/open the camera" << std::endl);
        return r;
    }

    r = libusb_claim_interface(devh_, 0);
    if (r < 0) {
        ROS_ERROR_STREAM("usb_claim_interface error " << r << std::endl);
        libusb_close(devh_);
        libusb_exit(NULL);
        return r;
    }

    init();

    ROS_INFO_STREAM("device::open. exit.");
    return result;
}

int BaseCamera::init() {
    int result = 0;

    ROS_INFO_STREAM("BaseCamera::init. Enter.");

    int r;

    r = libusb_control_transfer(devh_, 0x40, 0x22, height_, 0x0003, NULL, 0, 100);
    ROS_INFO_STREAM("set height. status " << r);

    r = libusb_control_transfer(devh_, 0x40, 0x22, width_, 0x0004, NULL, 0, 1000);
    ROS_INFO_STREAM("set width. status: " << r);

    ROS_INFO_STREAM("BaseCamera::init. Exit.");

    return result;
}

int BaseCamera::run(void) {
    return libusb_control_transfer(devh_, 0x40, 0xA8, 0x0000, 0x00, NULL, 0, 100);
}

int BaseCamera::GetImage(byte* buffer, std::size_t size) {
    int result = 0;

    int r;
    int actual;

    // Start.
    libusb_control_transfer(devh_, 0x40, 0xA8, 0x0000, 0x00, NULL, 0, 10);
    // Read.
    r = libusb_bulk_transfer(devh_, 0x82, buffer, size, &actual, 1000);
    // Stop.
    libusb_control_transfer(devh_, 0x40, 0xA9, 0x0000, 0x00, NULL, 0, 10);

    result = actual;

    if (libusb_error::LIBUSB_SUCCESS != r) {
        // TODO: handle correctly error.
        ROS_ERROR_STREAM("Error during read data from bulk. error=" << r << std::endl);
        return r;
    }
    if (size != actual) {
        ROS_ERROR_STREAM(
                "Actual number of read byte is not as expected. actual=" << actual << std::endl);
        result = -1;
    }

    return result;
}

int BaseCamera::stop() {
    // Stop camera
    return libusb_control_transfer(devh_, 0x40, 0xA9, 0x0000, 0x00, NULL, 0, 1000);
}

int BaseCamera::close() {
    int result = 0;

    ROS_INFO_STREAM("device::close. Enter.");

    if (NULL != devh_) {
        libusb_release_interface(devh_, 0);
        libusb_close(devh_);
        libusb_exit(NULL);
        devh_ = NULL;
    }
    ROS_INFO_STREAM("device::close. Exit.");

    return result;
}
} // namespace ros_camera

