/*
 * exception.hpp
 *
 *  Created on: Dec 5, 2017
 */

#pragma once

#include <stdexcept>
#include <string>

namespace ros_camera {

/**
 * @brief ROS camera device exception.
 *
 */
class DeviceError: public std::runtime_error {
public:
    explicit DeviceError(const std::string &cause) :
            std::runtime_error(cause) {
    }
};

}  // namespace ros_camera
