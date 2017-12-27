

#include "../include/ros_camera/controller.h"
#include "ros_camera/exception.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_camera");
    ros::NodeHandle private_node("~");
    ros_camera::Controller camera(private_node, private_node);

    try {
        camera.setup();
        camera.run();

        ros::spin();
    } catch (ros_camera::DeviceError &e) {
        ROS_ERROR_STREAM("camera open failed: " << e.what());
        return 1;
    }

    return 0;
}
