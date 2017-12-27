/*
 * device.h
 *
 *  Created on: Dec 5, 2017
 *
 *  Interface to camera device.
 *  TODO: now to keep it easy we implement the interface here.
 *  TODO: need to find better name for both file and class names
 */

#include <cstdio>
#include <libusb.h>

#include "wqueue.h"
#include "imgframe.h"
#include "commbase.h"

namespace ros_camera {

class BaseCamera {

public:
    BaseCamera(int height, int width);
    ~BaseCamera();

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
    int GetImage(byte* buffer, std::size_t size);

    /**
     * @brief Close camera.
     *
     * @return 0 in success
     */
    int close();

private:
    libusb_device_handle* devh_;
    int height_;
    int width_;
};

} // namespace ros_camera
