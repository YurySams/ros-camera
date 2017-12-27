/*
 * arducam_mt9v034.cpp
 *
 *  Created on: Dec 13, 2017
 */

#include <iostream>

#include <ros/ros.h>

#include "ros_camera/arducam_mt9v034.h"

namespace ros_camera {

ARDUCAM_MT9V034::ARDUCAM_MT9V034() {

}

ARDUCAM_MT9V034::~ARDUCAM_MT9V034() {

    ROS_INFO_STREAM("ARDUCAM_MT9V034::Destructor. Enter.");
    close();
    ROS_INFO_STREAM("ARDUCAM_MT9V034::Destructor. Exit.");
}

int ARDUCAM_MT9V034::open() {
    int result = 0;
    int r;

    ROS_INFO_STREAM("device::open. Enter.");
    r = libusb_init(NULL);
    if (libusb_error::LIBUSB_SUCCESS != r) {
        // TODO: handle correctly error.
        ROS_ERROR_STREAM("Cannot init usblib. error=" << r);
        return r;
    }

    // TODO: to find place for 0x52CB and 0x52CB.
    devh_ = libusb_open_device_with_vid_pid(NULL, 0x52CB, 0x52CB);

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

int ARDUCAM_MT9V034::run() {
    int result = 0;
    // Do nothing for this device.
    return result;
}

/*
 int ARDUCAM_MT9V034::GetImage(byte* buffer, std::size_t size) {
 auto const PACK_SIZE = 4096;
 //    auto const PACK_SIZE = 8192;
 //    auto const PACK_SIZE = size;

 int pack_number = size / PACK_SIZE;
 int left = size;
 int actual;
 int r;
 int pack_size = PACK_SIZE;

 while (left > 0) {
 r = libusb_bulk_transfer(devh_, 0x82, buffer, pack_size, &actual, 10000);
 ROS_INFO_STREAM("libusb_bulk_transfer. r=" << r << "; actual size=" << actual << "; left=" << left);

 buffer = buffer + actual;
 left = left - actual;
 pack_size = left >= PACK_SIZE ? PACK_SIZE : left;
 }
 }
 */

int ARDUCAM_MT9V034::GetImage(byte* buffer, std::size_t size, void* user_data,
        image_ready_cb image_ready) {
    int result;
    user_data_ = user_data;
    buffer_ = buffer;
    current_ = 0;
    expected_ = size;
    image_ready_ = image_ready;
    for (int i = 0; i < 32; i++) {
        struct libusb_transfer* transfer = create_transfer(4096);
//        struct libusb_transfer* transfer = create_transfer(8192);

        libusb_submit_transfer(transfer);
    }

    while (1) {
        result = libusb_handle_events(NULL);
        if (result < 0) {
            ROS_ERROR_STREAM("GetImage. error: " << result);
            break;
        }
    }
}

void ARDUCAM_MT9V034::read_callback(struct libusb_transfer *transfer) {
    int result;
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        /* Success! Handle data received */
//        transfer->num_iso_packets
        ROS_INFO_STREAM("read_callback. actual=" << transfer->actual_length << " current: " << current_ << "; isync=" << transfer->num_iso_packets);
        int actual = transfer->actual_length;
        int left = expected_ - current_;

        if (actual > 0) {
            if (actual <= left) {
                memcpy(buffer_ + current_, transfer->buffer, actual);
                current_ += actual;
                if (current_ == expected_) {
                    image_ready_(user_data_);
                    current_ = 0;
                }
            } else {
                memcpy(buffer_ + current_, transfer->buffer, left);
                image_ready_(user_data_);
                current_ = actual - left;
                memcpy(buffer_ , transfer->buffer + left, current_);
            }
        }

        /* Re-submit the transfer object. */
        result = libusb_submit_transfer(transfer);
        if (result != 0) {
            printf("submitting. error code: %d\n", result);
        }

    } else {
        printf("Error: %d\n", transfer->status);
    }

}

static void read_callback_static(struct libusb_transfer *transfer) {
    if (transfer->user_data != NULL) {
        ((ARDUCAM_MT9V034*) transfer->user_data)->read_callback(transfer);
    }
}

struct libusb_transfer * ARDUCAM_MT9V034::create_transfer(int size) {
    struct libusb_transfer* transfer;
    unsigned char* buf;
    buf = new byte[size];

    transfer = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(transfer, devh_, 0x82, buf, size, read_callback_static, this, 1000);
    return transfer;
}

int ARDUCAM_MT9V034::stop() {
    int result = 0;
// Do nothing here.
    return result;
}

int ARDUCAM_MT9V034::close() {
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

static unsigned char read_buffer[64];
int ARDUCAM_MT9V034::init() {
    ROS_INFO_STREAM("device::init. Enter.");
    int result = 0;
    int r;

    {
        // Request 240
        r = libusb_control_transfer(devh_, 0x40, 0xF0, 0x0000, 0x0000, NULL, 0, 100);
    }

    {
        // Request 241
        r = libusb_control_transfer(devh_, 0xC0, 0xF1, 0x0400, 0x0000, read_buffer, 4, 100);
    }

    {
        // Request 242
        read_buffer[0] = 0x03;
        read_buffer[1] = 0x07;
        read_buffer[2] = 0x02;
        read_buffer[3] = 0x00;
        read_buffer[4] = 0x00;
        read_buffer[5] = 0x00;
        read_buffer[6] = 0x1E;
        read_buffer[7] = 0x2D;
        r = libusb_control_transfer(devh_, 0xC0, 0xF2, 0x0800, 0x0000, read_buffer, 8, 100);
    }

    {
        // Request 241
        r = libusb_control_transfer(devh_, 0xC0, 0xF1, 0x0700, 0x0000, read_buffer, 7, 100);
    }

    {
        // Request 242
        read_buffer[0] = 0x03;
        read_buffer[1] = 0x07;
        read_buffer[2] = 0x02;
        read_buffer[3] = 0x00;
        read_buffer[4] = 0x00;
        read_buffer[5] = 0x00;
        read_buffer[6] = 0x18;
        read_buffer[7] = 0xAD;
        r = libusb_control_transfer(devh_, 0xC0, 0xF2, 0x0800, 0x0000, read_buffer, 8, 100);
    }

    {
        // Request 241
        r = libusb_control_transfer(devh_, 0xC0, 0xF1, 0x0700, 0x0000, read_buffer, 7, 100);
    }

    {
        // Request 242
        read_buffer[0] = 0x03;
        read_buffer[1] = 0x07;
        read_buffer[2] = 0x02;
        read_buffer[3] = 0x00;
        read_buffer[4] = 0x00;
        read_buffer[5] = 0x00;
        read_buffer[6] = 0x11;
        read_buffer[7] = 0x2D;
        r = libusb_control_transfer(devh_, 0xC0, 0xF2, 0x0800, 0x0000, read_buffer, 8, 100);
    }

    {
        // Request 241
        r = libusb_control_transfer(devh_, 0xC0, 0xF1, 0x0700, 0x0000, read_buffer, 7, 100);
    }

    {
        // Request 242
        read_buffer[0] = 0x03;
        read_buffer[1] = 0x1B;
        read_buffer[2] = 0x16;
        read_buffer[3] = 0x01;
        read_buffer[4] = 0x00;
        read_buffer[5] = 0x00;
        read_buffer[6] = 0xBA;
        read_buffer[7] = 0x48;

        read_buffer[8] = 0x24;
        read_buffer[9] = 0x20;
        read_buffer[10] = 0xFC;
        read_buffer[11] = 0x43;
        read_buffer[12] = 0x82;
        read_buffer[13] = 0xEE;
        read_buffer[14] = 0x4D;
        read_buffer[15] = 0x7A;

        read_buffer[16] = 0xC8;
        read_buffer[17] = 0xD4;
        read_buffer[18] = 0xB9;
        read_buffer[19] = 0x42;
        read_buffer[20] = 0x90;
        read_buffer[21] = 0x3C;
        read_buffer[22] = 0x39;
        read_buffer[23] = 0xD7;

        read_buffer[24] = 0x88;
        read_buffer[25] = 0x32;
        read_buffer[26] = 0x9C;
        read_buffer[27] = 0xEF;

        r = libusb_control_transfer(devh_, 0xC0, 0xF2, 0x1C00, 0x0000, read_buffer, 28, 100);
    }

    {
        r = libusb_control_transfer(devh_, 0xC0, 0xF1, 0x2300, 0x0400, read_buffer, 35, 100);
    }

    {
        // Request 242
        read_buffer[0] = 0x03;
        read_buffer[1] = 0x27;
        read_buffer[2] = 0x08;
        read_buffer[3] = 0x41;
        read_buffer[4] = 0x00;
        read_buffer[5] = 0x00;
        read_buffer[6] = 0xCB;
        read_buffer[7] = 0xF9;

        read_buffer[8] = 0xBC;
        read_buffer[9] = 0x39;
        read_buffer[10] = 0x83;
        read_buffer[11] = 0x7A;
        read_buffer[12] = 0x90;
        read_buffer[13] = 0x53;
        read_buffer[14] = 0x7A;
        read_buffer[15] = 0x78;

        read_buffer[16] = 0x4A;
        read_buffer[17] = 0x92;
        read_buffer[18] = 0x41;
        read_buffer[19] = 0x15;
        read_buffer[20] = 0xCD;
        read_buffer[21] = 0x7E;
        read_buffer[22] = 0x85;
        read_buffer[23] = 0xED;

        read_buffer[24] = 0xD6;
        read_buffer[25] = 0xBE;
        read_buffer[26] = 0xC1;
        read_buffer[27] = 0x68;
        read_buffer[28] = 0x5E;
        read_buffer[29] = 0x95;
        read_buffer[30] = 0xE5;
        read_buffer[31] = 0xB6;

        read_buffer[32] = 0xFC;
        read_buffer[33] = 0xA9;
        read_buffer[34] = 0x2E;
        read_buffer[35] = 0x18;
        read_buffer[36] = 0xED;
        read_buffer[37] = 0x2B;
        read_buffer[38] = 0xC3;
        read_buffer[39] = 0x46;
        r = libusb_control_transfer(devh_, 0xC0, 0xF2, 0x1C00, 0x0000, read_buffer, 40, 100);
    }

    {
        r = libusb_control_transfer(devh_, 0xC0, 0xF1, 0x2300, 0x0400, read_buffer, 35, 100);
    }

// [0x01, 0x0001]
    {
        unsigned char data[2] = { 0x00, 0x01 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0100, data, 2, 100);
    }
    {
        // [0x02, 0x0004]
        unsigned char data[2] = { 0x00, 0x04 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0200, data, 2, 100);
    }
    {
        // [0x03, 0x01E0],
        unsigned char data[2] = { 0x01, 0xE0 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0300, data, 2, 100);
    }
    {
        // [0x04, 640],
        // [0x04, 752],
        unsigned char data[2] = { 0x02, 0x80 };
//        unsigned char data[2] = { 0x02, 0xF0 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0400, data, 2, 100);
    }
    {
        // [0x05, 0x005E],
        unsigned char data[2] = { 0x00, 0x5E };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0500, data, 2, 100);
    }
    {
        // [0x06, 0x0039],
        unsigned char data[2] = { 0x00, 0x39 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0600, data, 2, 100);
    }
    {
        /// [0x07, 0x0188],
        unsigned char data[2] = { 0x01, 0x88 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0700, data, 2, 100);
    }
    {
        // [0x08, 0x0190],
        unsigned char data[2] = { 0x01, 0x90 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0800, data, 2, 100);
    }
    {
        // [0x09, 0x01BD],
        unsigned char data[2] = { 0x01, 0xBD };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0900, data, 2, 100);
    }
    {
        // [0x0A, 0x0164],
        unsigned char data[2] = { 0x01, 0x64 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0A00, data, 2, 100);
    }
    {
        // [0x0B, 0x01C2],
        unsigned char data[2] = { 0x01, 0xC2 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0B00, data, 2, 100);
    }
    {
        // [0x0C, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0C00, data, 2, 100);
    }
    {
        // [0x0D, 0x0300],
        unsigned char data[2] = { 0x03, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0D00, data, 2, 100);
    }
    {
        //[0x0E, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0E00, data, 2, 100);
    }
    {
//        #[0x0F, 0x0100],
        // [0x10, 0x0040],
        unsigned char data[2] = { 0x00, 0x40 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1000, data, 2, 100);
    }
    {
        // [0x11, 0x8042],
        unsigned char data[2] = { 0x80, 0x40 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1100, data, 2, 100);
    }
    {
        // [0x12, 0x0022],
        unsigned char data[2] = { 0x00, 0x22 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1200, data, 2, 100);
    }
    {
        // [0x13, 0x2D2E],
        unsigned char data[2] = { 0x2D, 0x2E };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1300, data, 2, 100);
    }
    {
        // [0x14, 0x0E02],
        unsigned char data[2] = { 0xE0, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1400, data, 2, 100);
    }
    {
        //[0x15, 0x0E32],
        unsigned char data[2] = { 0xE0, 0x32 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1500, data, 2, 100);
    }
    {
        // [0x16, 0x2802],
        unsigned char data[2] = { 0x28, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1600, data, 2, 100);
    }
    {
        //[0x17, 0x3E38],
        unsigned char data[2] = { 0x3E, 0x38 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1700, data, 2, 100);
    }
    {
        // [0x18, 0x3E38],
        unsigned char data[2] = { 0x3E, 0x38 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1800, data, 2, 100);
    }
    {
        //[0x19, 0x2802],
        unsigned char data[2] = { 0x28, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1900, data, 2, 100);
    }
    {
        // [0x1A, 0x0428],
        unsigned char data[2] = { 0x04, 0x28 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1A00, data, 2, 100);
    }
    {
        // [0x1B, 0x0000],
        unsigned char data[2] = { 0x00, 0x01 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1B00, data, 2, 100);
    }
    {
        // [0x1C, 0x0302],
        unsigned char data[2] = { 0x03, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1C00, data, 2, 100);
    }
    {
        // [0x1D, 0x0040],
        unsigned char data[2] = { 0x00, 0x40 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1D00, data, 2, 100);
    }
    {
        // [0x1E, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1E00, data, 2, 100);
    }
    {
        //[0x1F, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x1F00, data, 2, 100);
    }
    {
        // [0x20, 0x03C7],
        unsigned char data[2] = { 0x03, 0xC7 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2000, data, 2, 100);
    }
    {
        // [0x21, 0x0020],
        unsigned char data[2] = { 0x00, 0x20 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2100, data, 2, 100);
    }
    {
        // [0x22, 0x0020],
        unsigned char data[2] = { 0xE0, 0x20 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2200, data, 2, 100);
    }
    {
        // [0x23, 0x0010],
        unsigned char data[2] = { 0x00, 0x10 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2300, data, 2, 100);
    }
    {
        // [0x24, 0x001B],
        unsigned char data[2] = { 0x00, 0x1B };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2400, data, 2, 100);
    }
    {
        //[0x25, 0x001A],
        unsigned char data[2] = { 0x00, 0x1A };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2500, data, 2, 100);
    }
    {
        //[0x26, 0x0004],
        unsigned char data[2] = { 0x00, 0x04 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2600, data, 2, 100);
    }
    {
        // [0x27, 0x000C],
        unsigned char data[2] = { 0x00, 0x0C };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2700, data, 2, 100);
    }
    {
        //[0x28, 0x0010],
        unsigned char data[2] = { 0x00, 0x10 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2800, data, 2, 100);
    }
    {
        //[0x29, 0x0010],
        unsigned char data[2] = { 0x00, 0x10 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2900, data, 2, 100);
    }
    {
        //[0x2A, 0x0020],
        unsigned char data[2] = { 0x00, 0x20 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2A00, data, 2, 100);
    }
    {
        //[0x2B, 0x0003],
        unsigned char data[2] = { 0x00, 0x03 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2B00, data, 2, 100);
    }
    {
        //[0x2C, 0x0004],
        unsigned char data[2] = { 0x00, 0x04 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2C00, data, 2, 100);
    }
    {
        //[0x2D, 0x0004],
        unsigned char data[2] = { 0x00, 0x04 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2D00, data, 2, 100);
    }
    {
        //[0x2E, 0x0007],
        unsigned char data[2] = { 0x00, 0x07 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2E00, data, 2, 100);
    }
    {
        // [0x2F, 0x0003],
        unsigned char data[2] = { 0x00, 0x03 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x2F00, data, 2, 100);
    }
    {
        //[0x30, 0x0003],
        unsigned char data[2] = { 0x00, 0x03 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3000, data, 2, 100);
    }
    {
        // [0x31, 0x001F],
        unsigned char data[2] = { 0x00, 0x1F };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3100, data, 2, 100);
    }
    {
        // [0x32, 0x001A],
        unsigned char data[2] = { 0x00, 0x1A };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3200, data, 2, 100);
    }
    {
        // [0x33, 0x0012],
        unsigned char data[2] = { 0x00, 0x12 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3300, data, 2, 100);
    }
    {
        // [0x34, 0x0003],
        unsigned char data[2] = { 0x00, 0x03 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0300, data, 2, 100);
    }
    {
        // [0x35, 0x0020],
        unsigned char data[2] = { 0x00, 0x20 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3500, data, 2, 100);
    }
    {
        //[0x36, 0x0010],
        unsigned char data[2] = { 0x00, 0x10 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3600, data, 2, 100);
    }
    {
        //[0x37, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3700, data, 2, 100);
    }
    {
        //[0x38, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3800, data, 2, 100);
    }
    {
        //[0x39, 0x0025],
        unsigned char data[2] = { 0x00, 0x25 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3900, data, 2, 100);
    }
    {
        //[0x3A, 0x0020],
        unsigned char data[2] = { 0x00, 0x20 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3A00, data, 2, 100);
    }
    {
        // [0x3B, 0x0003],
        unsigned char data[2] = { 0x00, 0x03 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3B00, data, 2, 100);
    }
    {
        //[0x3C, 0x0003],
        unsigned char data[2] = { 0x00, 0x03 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x3C00, data, 2, 100);
    }
    {
        //[0x46, 0x231D],
        unsigned char data[2] = { 0x23, 0x1D };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x4600, data, 2, 100);
    }
    {
        //[0x47, 0x0080],
        unsigned char data[2] = { 0x00, 0x80 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x4700, data, 2, 100);
    }
    {
        // [0x4C, 0x0002],
        unsigned char data[2] = { 0x00, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x4C00, data, 2, 100);
    }
    {
        // [0x70, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x7000, data, 2, 100);
    }
    {
        // [0x71, 0x002A],
        unsigned char data[2] = { 0x00, 0x2A };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x7100, data, 2, 100);
    }
    {
        // [0x72, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x7200, data, 2, 100);
    }
    {
        //[0x7F, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x7F00, data, 2, 100);
    }
    {
        // [0x80, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8000, data, 2, 100);
    }
    {
        // [0x81, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8100, data, 2, 100);
    }
    {
        //[0x82, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8200, data, 2, 100);
    }
    {
        //[0x83, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x800, data, 2, 100);
    }
    {
        //[0x84, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8400, data, 2, 100);
    }
    {
        // [0x85, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8500, data, 2, 100);
    }
    {
        //[0x86, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8600, data, 2, 100);
    }
    {
        //[0x87, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8700, data, 2, 100);
    }
    {
        // [0x88, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8800, data, 2, 100);
    }
    {
        // [0x89, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8900, data, 2, 100);
    }
    {
        // [0x8A, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8A00, data, 2, 100);
    }
    {
        // [0x8B, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8B00, data, 2, 100);
    }
    {
        // [0x8C, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8C00, data, 2, 100);
    }
    {
        // [0x8D, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8D00, data, 2, 100);
    }
    {
        //[0x8E, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8E00, data, 2, 100);
    }
    {
        // [0x8F, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x8F00, data, 2, 100);
    }
    {
        // [0x90, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9000, data, 2, 100);
    }
    {
        // [0x91, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9100, data, 2, 100);
    }
    {
        // [0x92, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9200, data, 2, 100);
    }
    {
        // [0x93, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9300, data, 2, 100);
    }
    {
        // [0x94, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9400, data, 2, 100);
    }
    {
        // [0x95, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9500, data, 2, 100);
    }
    {
        // [0x96, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9600, data, 2, 100);
    }
    {
        // [0x97, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9700, data, 2, 100);
    }
    {
        // [0x98, 0x04F4],
        unsigned char data[2] = { 0x04, 0xF4 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9800, data, 2, 100);
    }
    {
        //[0x99, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9900, data, 2, 100);
    }
    {
        //[0x9A, 0x0096],
        unsigned char data[2] = { 0x00, 0x96 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9A00, data, 2, 100);
    }
    {
        // [0x9B, 0x012C],
        unsigned char data[2] = { 0x01, 0x2C };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9B00, data, 2, 100);
    }
    {
        //[0x9C, 0x01C2],
        unsigned char data[2] = { 0x01, 0xC2 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9C00, data, 2, 100);
    }
    {
        //[0x9D, 0x0258],
        unsigned char data[2] = { 0x02, 0x58 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9D00, data, 2, 100);
    }
    {
        //[0x9E, 0x02F0],
        unsigned char data[2] = { 0x02, 0xF0 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9E00, data, 2, 100);
    }
    {
        //[0x9F, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x9F00, data, 2, 100);
    }
    {
        //[0xA0, 0x0060],
        unsigned char data[2] = { 0x00, 0x60 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xA000, data, 2, 100);
    }
    {
        //[0xA1, 0x00C0],
        unsigned char data[2] = { 0x00, 0xC0 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xA100, data, 2, 100);
    }
    {
        //[0xA2, 0x0120],
        unsigned char data[2] = { 0x01, 0x20 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xA200, data, 2, 100);
    }
    {
        // [0xA3, 0x0180],
        unsigned char data[2] = { 0x01, 0x80 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xA300, data, 2, 100);
    }
    {
        // [0xA4, 0x01E0],
        unsigned char data[2] = { 0x01, 0xE0 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xA400, data, 2, 100);
    }
    {
        //[0xA5, 0x003A],
        unsigned char data[2] = { 0x00, 0x3A };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xA500, data, 2, 100);
    }
    {
        //[0xA6, 0x0002],
        unsigned char data[2] = { 0x00, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xA600, data, 2, 100);
    }
    {
        // [0xA8, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xA800, data, 2, 100);
    }
    {
        // [0xA9, 0x0002],
        unsigned char data[2] = { 0x00, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xA900, data, 2, 100);
    }
    {
        // [0xAA, 0x0002],
        unsigned char data[2] = { 0x00, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xAA00, data, 2, 100);
    }
    {
        // [0xAB, 0x0040],
        unsigned char data[2] = { 0x00, 0x40 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xAB00, data, 2, 100);
    }
    {
        // [0xAC, 0x0001],
        unsigned char data[2] = { 0x00, 0x01 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xAC00, data, 2, 100);
    }
    {
        // [0xAD, 0x01E0],
        unsigned char data[2] = { 0x01, 0xE0 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xAD00, data, 2, 100);
    }
    {
        // [0xAE, 0x0014],
        unsigned char data[2] = { 0x00, 0x14 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xAE00, data, 2, 100);
    }
    {
        //[0xAF, 0x0003],
        unsigned char data[2] = { 0x00, 0x03 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xAF00, data, 2, 100);
    }
    {
        //[0xB0, 0xABE0],
        unsigned char data[2] = { 0xAB, 0xE0 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xB000, data, 2, 100);
    }
    {
        //[0xB1, 0x0002],
        unsigned char data[2] = { 0x00, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xB100, data, 2, 100);
    }
    {
        //[0xB2, 0x0010],
        unsigned char data[2] = { 0x00, 0x10 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xB200, data, 2, 100);
    }
    {
        //[0xB3, 0x0010],
        unsigned char data[2] = { 0x00, 0x10 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xB300, data, 2, 100);
    }
    {
        // [0xB4, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xB400, data, 2, 100);
    }
    {
        // [0xB5, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xB500, data, 2, 100);
    }
    {
        // [0xB6, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xB600, data, 2, 100);
    }
    {
        // [0xB7, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xB700, data, 2, 100);
    }
    {
        // [0xBF, 0x0016],
        unsigned char data[2] = { 0x00, 0x16 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xBF00, data, 2, 100);
    }
    {
        // [0xC0, 0x000A],
        unsigned char data[2] = { 0x00, 0x0A };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xC000, data, 2, 100);
    }
    {
        //[0xC2, 0x18D0],
        unsigned char data[2] = { 0x18, 0xD0 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xC200, data, 2, 100);
    }
    {
        //#[0xC3, 0x007F],
        //[0xC4, 0x007F],
        unsigned char data[2] = { 0x00, 0x7F };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xC400, data, 2, 100);
    }
    {
        //[0xC5, 0x007F],
        unsigned char data[2] = { 0x00, 0x7F };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xC500, data, 2, 100);
    }
    {
        // [0xC6, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xC600, data, 2, 100);
    }
    {
        //[0xC7, 0x4416],
        unsigned char data[2] = { 0x44, 0x16 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xC700, data, 2, 100);
    }
    {
        // [0xC8, 0x4421],
        unsigned char data[2] = { 0x44, 0x21 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xC800, data, 2, 100);
    }
    {
        // [0xC9, 0x0002],
        unsigned char data[2] = { 0x00, 0x02 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xC900, data, 2, 100);
    }
    {
        // [0xCA, 0x0004],
        unsigned char data[2] = { 0x00, 0x04 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xCA00, data, 2, 100);
    }
    {
        // [0xCB, 0x01E0],
        unsigned char data[2] = { 0x01, 0xE0 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xCB00, data, 2, 100);
    }
    {
        // [0xCC, 0x02EE],
        unsigned char data[2] = { 0x02, 0xEE };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xCC00, data, 2, 100);
    }
    {
        //[0xCD, 0x0100],
        unsigned char data[2] = { 0x10, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xCD00, data, 2, 100);
    }
    {
        //[0xCE, 0x0100],
        unsigned char data[2] = { 0x10, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xCE00, data, 2, 100);
    }
    {
        //[0xCF, 0x0190],
        unsigned char data[2] = { 0x01, 0x90 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xCF00, data, 2, 100);
    }
    {
        //[0xD0, 0x01BD],
        unsigned char data[2] = { 0x01, 0xBD };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD000, data, 2, 100);
    }
    {
        //[0xD1, 0x0064],
        unsigned char data[2] = { 0x00, 0x64 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD100, data, 2, 100);
    }
    {
        //[0xD2, 0x01C2],
        unsigned char data[2] = { 0x01, 0xC2 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD200, data, 2, 100);
    }
    {
        //[0xD3, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD300, data, 2, 100);
    }
    {
        //[0xD4, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD400, data, 2, 100);
    }
    {
        // [0xD5, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD500, data, 2, 100);
    }
    {
        // [0xD6, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD600, data, 2, 100);
    }
    {
        //[0xD7, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD700, data, 2, 100);
    }
    {
        //[0xD8, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD800, data, 2, 100);
    }
    {
        //[0xD9, 0x0000],
        unsigned char data[2] = { 0x00, 0x00 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xD900, data, 2, 100);
    }
    {
        //[0x07, 904],
        unsigned char data[2] = { 0x03, 0x88 };
        r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0x0700, data, 2, 100);
    }
    /* There is no 0xFFFF index in backtrace of arducam.
     {
     //[0xFFFF, 0xFFFF]
     unsigned char data[2] = { 0xFF, 0xFF };
     r = libusb_control_transfer(devh_, 0x40, 0xD5, 0x9000, 0xFFFF, data, 2, 100);
     }
     */
    ROS_INFO_STREAM("device::init. Exit.");
    return result;
}

} // namespace ros_camera

