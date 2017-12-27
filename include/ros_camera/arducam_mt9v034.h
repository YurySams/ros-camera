/*
 * arducam_mt9v034.h
 *
 *  Created on: Dec 13, 2017
 */

#pragma once

#include <cstdio>
#include <libusb.h>

#include "wqueue.h"
#include "imgframe.h"
#include "commbase.h"

namespace ros_camera {

typedef void (*image_ready_cb)(void* arg);


class ARDUCAM_MT9V034 {
public:
    ARDUCAM_MT9V034();
    ~ARDUCAM_MT9V034();

    struct libusb_transfer* create_transfer(int size);

    void read_callback(struct libusb_transfer *transfer);

    /**
     * @brief Open camera and init it. Camera should be closed before exit.
     *
     * @return 0 if success; otherwise code of the error
     */
    int open();

    /**
     * @brief Init opened camera
     *
     * @return 0 if success; otherwise code of the error
     */
    int init();

    /**
     * @brief Run camera
     *
     * @return 0 if success; otherwise code of the error
     */
    int run();

    /**
     * @brief Stop shooting
     *
     * @return 0 if success; otherwise code of the error
     */
    int stop();

    /**
     * @brief Read one frame from camera
     *
     * @return 0 if success; otherwise code of the error
     */
    int GetImage(byte* buffer, std::size_t size, void* user_data, image_ready_cb image_ready);

    /**
     * @brief Close camera.
     *
     * @return 0 in success
     */
    int close();

private:
    libusb_device_handle* devh_;
    void* user_data_;
    byte* buffer_;
    int current_;
    int expected_;

    image_ready_cb image_ready_;
};
} // namespace ros_camera
