/*
 * Copyright (C) 2010 Philipp Robbel
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implements basic Kinect driver.
 *
 * NOTE: In its current form, the camera needs a few seconds of startup time
 * before it works. If launched too early, garbled images may result.
 */

#include "KinectDeviceDriver.h"
using namespace yarp::dev;

// Define constants here
const int KinectDeviceDriver::width_ = 640;
const int KinectDeviceDriver::height_ = 480;
const double KinectDeviceDriver::maxRange_ = 5.; // previous author said: "XXX not sure"
const double KinectDeviceDriver::horizontalFOV_= 57.;
const double KinectDeviceDriver::verticalFOV_ = 43.;

//! Global pointer for callback functions
KinectDeviceDriver *kinect;

//! Depth callback for libfreenect's C interface
static void depthimg(freenect_device *dev, void *data, uint32_t timestamp) {
  kinect->depthImgCb(dev, data, timestamp);
}

//! Rgb callback for libfreenect's C interface
static void rgbimg(freenect_device *dev, void *data, uint32_t timestamp) {
  kinect->rgbImgCb(dev, data, timestamp);
}

KinectDeviceDriver::KinectDeviceDriver() {

    rgbBuf_ = new uint8_t[width_*height_*3];
    depthBuf_ = new uint16_t[width_*height_];
    depthSent_ = true; // no new data available yet
    rgbSent_ = true;
    fn_ctx = NULL;
    fn_dev = NULL;
}

KinectDeviceDriver::~KinectDeviceDriver() {
    close();
    // clear up internal memory buffers
    if(rgbBuf_) { delete [] rgbBuf_; rgbBuf_ = 0; }
    if(depthBuf_) { delete [] depthBuf_; depthBuf_ = 0; }
}

bool KinectDeviceDriver::open(yarp::os::Searchable &config) {
    return open(cfg_);
}

bool KinectDeviceDriver::open(const KinectDeviceDriverSettings & cfg) {
    kinect = this; // set global pointer

    // Initialize libfreenect
    int ret = freenect_init(&fn_ctx, NULL);
    if (ret < 0)
        return ret;

    // Show debug messages and use camera only
    freenect_set_log_level(fn_ctx, FREENECT_LOG_DEBUG);
    freenect_select_subdevices(fn_ctx, FREENECT_DEVICE_CAMERA);

    // Find out how many devices are connected
    int num_devices = ret = freenect_num_devices(fn_ctx);
    if (ret < 0)
        return false;
    if (num_devices == 0)
    {
        printf("[KinectDeviceDriver] No device found!\n");
        freenect_shutdown(fn_ctx);
        return false;
    }

    // Open the first device found
    ret = freenect_open_device(fn_ctx, &fn_dev, 0);
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }

    // Set depth and video modes
    ret = freenect_set_depth_mode(fn_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM));
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }
    ret = freenect_set_video_mode(fn_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }

    // Set frame callbacks.
    freenect_set_depth_callback(fn_dev, depthimg);
    freenect_set_video_callback(fn_dev, rgbimg);

    // Start depth and video.
    ret = freenect_start_depth(fn_dev);
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }
    ret = freenect_start_video(fn_dev);
    if (ret < 0)
    {
        freenect_shutdown(fn_ctx);
        return ret;
    }

  // start usb grabbing thread
  usbSpinThread_.setContext(fn_ctx);
  usbSpinThread_.start();
  return true;
}

bool KinectDeviceDriver::close() {
  usbSpinThread_.stop();

  // Stop everything and shutdown.
  freenect_stop_depth(fn_dev);
  freenect_stop_video(fn_dev);
  freenect_close_device(fn_dev);
  freenect_shutdown(fn_ctx);

  kinect = NULL;
  return true;
}

bool KinectDeviceDriver::getImagePair(yarp::os::PortablePair<yarp::sig::ImageOf<yarp::sig::PixelMono16>, yarp::sig::ImageOf<yarp::sig::PixelRgb> > &pair) {
    pair.head.resize(width_,height_);
    pair.body.resize(width_,height_);

    depthMutex_.wait();
    // must transfer row by row, since YARP may use padding
    for (int i=0; i<height_; i++) {
        // depth image (1 layer)
        memcpy((unsigned char *)(&pair.head.pixel(0,i)),&depthBuf_[width_*i],width_*sizeof(uint16_t));
    }
    depthSent_ = true;
    depthMutex_.post();

    rgbMutex_.wait();
    for (int i=0; i<height_; i++) {
        // rgb image (3 layers), requires PixelRgb return value
        memcpy((unsigned char *)(&pair.body.pixel(0,i)),&rgbBuf_[i*width_*3],width_*3);
    }
    rgbSent_ = true;
    rgbMutex_.post();

    return true;
}

void KinectDeviceDriver::depthImgCb(freenect_device* dev, void* data, uint32_t timestamp) {
    depthMutex_.wait();
    depthSent_ = false; // new depth data available
    //memcpy(depthBuf_, buf, width_*height_*sizeof(uint16_t));
    printf("Received depth frame at %d\n", timestamp);
    depthMutex_.post();
}

void KinectDeviceDriver::rgbImgCb(freenect_device *dev, void *data, uint32_t timestamp) {
    rgbMutex_.wait();
    rgbSent_ = false; // new rgb data available
    //memcpy(rgbBuf_, buf, width_*height_*3);
    printf("Received image frame at %d\n", timestamp);
    rgbMutex_.post();
}
