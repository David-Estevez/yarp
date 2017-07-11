/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino <alberto.cardellino@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef YARP_DEV_FRAMEGRABBERCONTROL2_H
#define YARP_DEV_FRAMEGRABBERCONTROL2_H

#include <yarp/os/ConstString.h>
#include <yarp/dev/FrameGrabberInterfaces.h>        // to include VOCAB definitions

/*! \file FrameGrabberControl2.h define common interfaces to discover
 * remote camera capabilities */

namespace yarp{
    namespace dev {
        class IFrameGrabberControls2;
    }
}

typedef enum {
    BUS_UNKNOWN = 0,
    BUS_FIREWIRE,
    BUS_USB,
    BUS_GIGE
} BusType;

typedef enum {
    MODE_UNKNOWN = 0,
    MODE_MANUAL,
    MODE_AUTO
} FeatureMode;

typedef struct {
    BusType busType;
    yarp::os::ConstString deviceDescription;
} CameraDescriptor;


#define VOCAB_FRAMEGRABBER_CONTROL2     VOCAB4('f','g','c','2')
#define VOCAB_CAMERA_DESCRIPTION        VOCAB4('c','a','m','d')
#define VOCAB_HAS                       VOCAB3('h','a','s')
#define VOCAB_FEATURE                   VOCAB4('f','e','a','t')
#define VOCAB_FEATURE2                  VOCAB4('f','e','a','2')
#define VOCAB_ONOFF                     VOCAB4('o','n','o','f')
#define VOCAB_AUTO                      VOCAB4('a','u','t','o')
#define VOCAB_MANUAL                    VOCAB3('m','a','n')
#define VOCAB_ONEPUSH                   VOCAB4('o','n','e','p')
#define VOCAB_ACTIVE                    VOCAB4('a','c','t','v')
#define VOCAB_MODE                      VOCAB4('m','o','d','e')

/**
 * This interface exports a list of general methods to discover the remote camera
 * capabilities. It also has the get/set feature methods so can be used in conjuntion
 * with former IFrameGrabberControls2 or as standalone.
 */
class YARP_dev_API yarp::dev::IFrameGrabberControls2
{
public:

    virtual ~IFrameGrabberControls2(){};

    yarp::os::ConstString busType2String(BusType type)
    {
        switch (type) {
            case BUS_FIREWIRE:
                return "FireWire";
            break;

            case BUS_USB:
                return "USB";
            break;

            case BUS_GIGE:
                return "GigE";

            default:
                return "bus type undefined";
            break;
        }
    }

    inline FeatureMode toFeatureMode(bool _auto)
    {
        return _auto ? MODE_AUTO : MODE_MANUAL;
    }
    /**
     * Get a basic description of the camera hw. This is mainly used to determine the
     * HW bus type in order to choose the corresponding interface for advanced controls.
     * @param device returns an identifier for the bus
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool getCameraDescription(CameraDescriptor *camera)=0;

    /**
     * Check if camera has the requested feature (saturation, brightness ... )
     * @param feature the identifier of the feature to check
     * @param hasFeature flag value: true if the feature is present, false otherwise
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool hasFeature(int feature, bool *hasFeature)=0;

    /**
     * Set the requested feature to a value (saturation, brightness ... )
     * @param feature the identifier of the feature to change
     * @param value new value of the feature
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool setFeature(int feature, double value)=0;

    /**
     * Get the current value for the requested feature.
     * @param feature the identifier of the feature to read
     * @param value  pointer to current value of the feature
     * @return returns true on success, false on failure.
     */
    virtual bool getFeature(int feature, double *value)=0;

    /**
     * Get the limits for the requested feature.
     * @param feature the identifier of the feature to read
     * @param minValue lower limit of the feature
     * @param maxValue upper limit of the feature
     * @return returns true on success, false on failure
     */
    virtual bool getFeatureLimits(int feature, double *minValue, double *maxValue)=0;

    /**
     * Set the requested feature to a value using 2 params (like white balance)
     * @param feature the identifier of the feature to change
     * @param value1  first param
     * @param value2  second param
     *
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool setFeature(int feature, double value1, double value2)=0;

    /**
     * Get the current value for the requested feature.
     * @param feature the identifier of the feature to read
     * @param value1  returns the current value of the feature
     * @param value2  returns the current value of the feature
     * @return returns true on success, false on failure.
     */
    virtual bool getFeature(int feature, double *value1, double *value2)=0;

    /**
     * Get the limits for the requested feature.
     * @param feature the identifier of the feature to read
     * @param minValue1 lower limit of the first param of the feature
     * @param maxValue1 upper limit of the first param of the feature
     * @param minValue2 lower limit of the second param of the feature
     * @param maxValue2 upper limit of the second param of the feature
     * @return returns true on success, false on failure.
     */
    virtual bool getFeatureLimits(int feature, double *minValue1, double *maxValue1, double *minValue2, double *maxValue2)=0;

    /**
     * Check if the camera has the ability to turn on/off the requested feature
     * @param feature the identifier of the feature to change
     * @param hasOnOff flag true if this feature can be turned on/off, false otherwise.
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool hasOnOff(int feature, bool *HasOnOff)=0;

    /**
     * Set the requested feature on or off
     * @param feature the identifier of the feature to change
     * @param onoff true to activate, off to deactivate the feature
     * @return returns true on success, false on failure.
     */
    virtual bool setActive(int feature, bool onoff)=0;

    /**
     * Get the current status of the feature, on or off
     * @param feature the identifier of the feature to check
     * @param isActive flag true if the feature is active, false otherwise
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool getActive(int feature, bool *isActive)=0;

    /**
     * Check if the requested feature has the 'auto' mode
     * @param feature the identifier of the feature to check
     * @param hasAuto flag true if the feature is has 'auto' mode, false otherwise
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool hasAuto(int feature, bool *hasAuto)=0;

    /**
     * Check if the requested feature has the 'manual' mode
     * @param feature the identifier of the feature to check
     * @param hasAuto flag true if the feature is has 'manual' mode, false otherwise
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool hasManual(int feature, bool *hasManual)=0;

    /**
     * Check if the requested feature has the 'onePush' mode
     * @param feature the identifier of the feature to check
     * @param hasAuto flag true if the feature is has 'onePush' mode, false otherwise
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool hasOnePush(int feature, bool *hasOnePush)=0;

    /**
     * Set the requested mode for the feature
     * @param feature the identifier of the feature to change
     * @param auto_onoff true to activate 'auto' mode, false to activate 'manual' mode
     * @return returns true on success, false on failure.
     */
    virtual bool setMode(int feature, FeatureMode mode)=0;

    /**
     * Get the current mode for the feature
     * @param feature the identifier of the feature to change
     * @param hasAuto flag true if the feature is has 'auto' mode, false otherwise
     * @return returns true if success, false otherwise (e.g. the interface is not implemented)
     */
    virtual bool getMode(int feature, FeatureMode *mode)=0;

    /**
     * Set the requested feature to a value (saturation, brightness ... )
     * @param feature the identifier of the feature to change
     * @param value new value of the feature, from 0 to 1 as a percentage of param range
     * @return returns true on success, false on failure.
     */
    virtual bool setOnePush(int feature)=0;
};

#endif  // YARP_DEV_FRAMEGRABBERCONTROL2_H
