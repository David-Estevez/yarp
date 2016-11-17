/*
 * Copyright (C) 2010 Philipp Robbel
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implements basic Kinect driver.
 *
 * NOTE: In its current form, the camera needs a few seconds of startup time
 * before it works. If launched too early, garbled images may result.
 */

#ifndef KINECTDEVICEDRIVER_H
#define KINECTDEVICEDRIVER_H

#include <memory.h>
#include <libusb.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/sig/Image.h>
#include <yarp/os/PortablePair.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>

#include "libfreenect.h"

namespace yarp {
  namespace dev {
    class KinectDeviceDriverSettings;
    class KinectDeviceDriver;
  }
}


/**
 * For documentation of settings, see above.
 */
class yarp::dev::KinectDeviceDriverSettings { };

class yarp::dev::KinectDeviceDriver : public DeviceDriver {
public:
  KinectDeviceDriver();

  virtual ~KinectDeviceDriver();

  virtual bool open(yarp::os::Searchable& config);

  /**
   * Configure the device.
   * @return true on success
   */
  bool open(const KinectDeviceDriverSettings & cfg);

  virtual bool close();

  /**
   * Get an image pair (depth and intensity images) from the kinect camera.
   * @return true/false upon success/failure
   */
  bool getImagePair(yarp::os::PortablePair<yarp::sig::ImageOf<yarp::sig::PixelMono16>, yarp::sig::ImageOf<yarp::sig::PixelRgb> > & pair);

  /**
   * Check whether new depth or rgb data have been retrieved from device.
   * Follow up with call to getImagePair() to obtain the actual images.
   */
  bool hasDepth() {
    return !depthSent_;
  }

  bool hasRgb() {
    return !rgbSent_;
  }

  /**
   * Return the height of each frame.
   * @return image height
   */
  int height() const {
    return height_;
  }

  /**
   * Return the width of each frame.
   * @return image width
   */
  int width() const {
    return width_;
  }

  /**
   * Return the maximum range of the sensor
   * @return sensor range
   */
  int maxRange() const {
    return maxRange_;
  }

  /**
   * Depth and rgb callbacks fill internal image buffers
   */
  void depthImgCb(freenect_device *dev, void *data, uint32_t timestamp);
  void rgbImgCb(freenect_device *dev, void *data, uint32_t timestamp);

private:
  //! driver settings (empty for now)
  yarp::dev::KinectDeviceDriverSettings cfg_;

  //! fixed device properties
  static const int width_;
  static const int height_;
  static const double maxRange_;
  static const double horizontalFOV_;
  static const double verticalFOV_;

  //! depth and rgb buffers from the kinect camera
  yarp::os::Semaphore depthMutex_;
  uint16_t *depthBuf_;
  yarp::os::Semaphore rgbMutex_;
  uint8_t *rgbBuf_;

  bool depthSent_;
  bool rgbSent_;

  //! Kinect-related objects
  freenect_context* fn_ctx;
  freenect_device* fn_dev;

  /**
   * Handle interaction with USB driver and fill internal image buffers
   */
  class USBThread : public yarp::os::Thread {
  public:
    USBThread() {
        fn_ctx = NULL;
    }

    void setContext(freenect_context* fn_ctx){
        this->fn_ctx = fn_ctx;
    }

    virtual void run() {
      // XXX Time::delay may interfere with USB read?!
      while(!isStopping() && freenect_process_events(fn_ctx) >= 0);
    }

  private:
    freenect_context* fn_ctx;
  } usbSpinThread_;
};

#endif
