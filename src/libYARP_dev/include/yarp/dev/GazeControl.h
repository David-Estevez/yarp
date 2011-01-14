// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2008 RobotCub Consortium
* Author: Ugo Pattacini
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef __YARPGAZECONTROLINTERFACES__
#define __YARPGAZECONTROLINTERFACES__

#include <yarp/os/Bottle.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/sig/Vector.h>

/*! \file GazeController.h define control board standard interfaces */

namespace yarp {
    namespace dev {
        class IGazeControl;
    }
}


/**
 * @ingroup dev_iface_motor
 *
 * Interface for a gaze controller.
 */
class YARP_dev_API yarp::dev::IGazeControl
{
public:
    /**
     * Destructor.
     */
    virtual ~IGazeControl() {}

    /**
    * Set the controller in tracking or non-tracking mode. [wait for
    * reply] 
    * @param f true for tracking mode, false otherwise. 
    * @return true/false on success/failure. 
    *  
    * @note In tracking mode when the controller reachs the target, 
    *       it keeps on running in order to mantain the gaze on
    *       target. In non-tracking mode the controller releases the
    *       control of the head as soon as the desired target is
    *       reached.
    */
    virtual bool setTrackingMode(const bool f)=0;

    /**
    * Get the current controller mode. [wait for reply]
    * @param f here is returned true if controller is in tracking 
    *         mode, false otherwise.
    * @return true/false on success/failure. 
    */
    virtual bool getTrackingMode(bool *f)=0;

    /**
    * Get the current fixation point. [do not wait for reply]
    * @param fp a 3-d vector which is filled with the actual 
    *         fixation point x,y,z (meters).
    * @return true/false on success/failure.
    */
    virtual bool getFixationPoint(yarp::sig::Vector &fp)=0;

    /**
    * Get the current gaze configuration in terms of azimuth and 
    * elevation angles wrt to the absolute reference frame along 
    * with the vergence. All angles are expressed in degrees [do not
    * wait for reply] 
    * @param ang a 3-d vector which is filled with the actual 
    *         angles azimuth/elevation/vergence (degrees).
    * @return true/false on success/failure. 
    *  
    * @note The absolute reference frame for the azimuth/elevation 
    *       couple is head-centered with the robot in rest
    *       configuration (i.e. torso and head angles zeroed).
    */
    virtual bool getAngles(yarp::sig::Vector &ang)=0;

    /**
    * Move the gaze to a specified fixation point in cartesian 
    * space. [do not wait for reply] 
    * @param fp a 3-d vector which contains the desired fixation 
    *         point where to look at x,y,z.
    * @return true/false on success/failure. 
    */
    virtual bool lookAtFixationPoint(const yarp::sig::Vector &fp)=0;

    /**
    * Move the gaze to a specified gazing angles configuration given
    * in the absolute reference frame [do not wait for reply] 
    * @param ang a 3-d vector which contains the desired gazing 
    *         angles (azimuth/elevation/vergence) expressed in the
    *         absolute reference frame (degrees).
    * @return true/false on success/failure. 
    *  
    * @note The absolute reference frame for the azimuth/elevation 
    *       couple is head-centered with the robot in rest
    *       configuration (i.e. torso and head angles zeroed). 
    */
    virtual bool lookAtAbsAngles(const yarp::sig::Vector &ang)=0;

    /**
    * Move the gaze to a specified gazing angles configuration given
    * in the relative reference frame [do not wait for reply] 
    * @param ang a 3-d vector which contains the desired gazing 
    *         angles (azimuth/elevation/vergence) expressed in the
    *         relative reference frame (degrees).
    * @return true/false on success/failure. 
    *  
    * @note The relative reference frame for the azimuth/elevation 
    *       couple is head-centered; the center of this frame is
    *       located in the middle of the baseline that connects the
    *       two eyes.
    */
    virtual bool lookAtRelAngles(const yarp::sig::Vector &ang)=0;

    /**
    * Move the gaze to a location specified by a pixel within the 
    * image plane [do not wait for reply] 
    * @param camSel selects the image plane: 0 for the left, 1 for 
    *              the right.
    * @param px a 2-d vector which contains the (u,v) coordinates of
    *           the pixel within the image plane.
    * @param z the z-component of the point in the eye's reference 
    *         frame (meters). A default value of 1.0 is assumed.
    * @return true/false on success/failure. 
    */
    virtual bool lookAtMonoPixel(const int camSel,
                                 const yarp::sig::Vector &px,
                                 const double z=1.0)=0;

    /**
    * Move the gaze to a location specified by two pixels 
    * representing the same 3-d point as seen from within both image
    * planes [do not wait for reply] 
    * @param pxl a 2-d vector which contains the (u,v) coordinates 
    *           of the pixel within the left image plane.
    * @param pxr a 2-d vector which contains the (u,v) coordinates 
    *           of the pixel within the right image plane.
    * @return true/false on success/failure. 
    *  
    * @note This strategy employs the monocular approach coupled 
    *       with a pid that varies the distance z incrementally
    *       according to the actual error: to achieve the target it
    *       is thus required to provide the visual feedback
    *       continuously.
    */
    virtual bool lookAtStereoPixels(const yarp::sig::Vector &pxl,
                                    const yarp::sig::Vector &pxr)=0;

    /**
    * Get the current trajectory duration for the neck actuators. 
    * [wait for reply] 
    * @param t time (seconds).
    * @return true/false on success/failure. 
    */
    virtual bool getNeckTrajTime(double *t)=0;

    /**
    * Get the current trajectory duration for the eyes actuators. 
    * [wait for reply] 
    * @param t time (seconds).
    * @return true/false on success/failure. 
    */
    virtual bool getEyesTrajTime(double *t)=0;

    /**
    * Get the current pose of the left eye frame. [wait for reply]
    * @param x a 3-d vector which is filled with the actual 
    *         position x,y,z (meters).
    * @param od a 4-d vector which is filled with the actual 
    * orientation using axis-angle representation xa, ya, za, theta 
    * (meters and radians). 
    * @return true/false on success/failure.
    */
    virtual bool getLeftEyePose(yarp::sig::Vector &x, yarp::sig::Vector &o)=0;

    /**
    * Get the current pose of the right eye frame. [wait for reply]
    * @param x a 3-d vector which is filled with the actual 
    *         position x,y,z (meters).
    * @param od a 4-d vector which is filled with the actual 
    * orientation using axis-angle representation xa, ya, za, theta 
    * (meters and radians). 
    * @return true/false on success/failure.
    */
    virtual bool getRightEyePose(yarp::sig::Vector &x, yarp::sig::Vector &o)=0;

    /**
    * Get the current pose of the head frame. [wait for reply]
    * @param x a 3-d vector which is filled with the actual 
    *         position x,y,z (meters).
    * @param od a 4-d vector which is filled with the actual 
    * orientation using axis-angle representation xa, ya, za, theta 
    * (meters and radians). 
    * @return true/false on success/failure. 
    *  
    * @note The center of the head frame is located in the middle of
    *       the baseline that connects the two eyes. The orientation
    *       of its frame is fixed with respect to the head with
    *       z-axis pointing forward, x-axis pointing rightward and
    *       y-axis pointing downward.
    */
    virtual bool getHeadPose(yarp::sig::Vector &x, yarp::sig::Vector &o)=0;

    /**
    * Get the joints target values where the controller is moving 
    * the system. [wait for reply] 
    * @param qdes a vector which is filled with the desired joints 
    *         values (degrees).
    * @return true/false on success/failure. 
    */
    virtual bool getJointsDesired(yarp::sig::Vector &qdes)=0;

    /**
    * Get the joints velocities commanded by the controller. [wait
    * for reply] 
    * @param qdot a vector which is filled with the joints 
    *         velocities (deg/s).
    * @return true/false on success/failure. 
    */
    virtual bool getJointsVelocities(yarp::sig::Vector &qdot)=0;

    /**
    * Returns the current options used by the stereo approach.
    * @param options is a property-like bottle containing the 
    *                current configuration employed by the internal
    *                pid.
    * @return true/false on success/failure. 
    *  
    * @note The returned bottle looks like as follows: 
    * (Kp (1 2 ...)) (Ki (1 2 ...)) (Kd (1 2 ...)) (Wp (...)) ... 
    * @note the satLim property is returned ordered by rows.
    */
    virtual bool getStereoOptions(yarp::os::Bottle &options)=0;

    /**
    * Set the duration of the trajectory for the neck actuators. 
    * [wait for reply] 
    * @param t time (seconds).
    * @return true/false on success/failure. 
    *  
    * @note The neck movements time cannot be set equal or lower 
    *       than the eyes movements time.
    */
    virtual bool setNeckTrajTime(const double t)=0;

    /**
    * Set the duration of the trajectory for the eyes actuators. 
    * [wait for reply] 
    * @param t time (seconds).
    * @return true/false on success/failure. 
    */
    virtual bool setEyesTrajTime(const double t)=0;

    /**
    * Update the options used by the stereo approach.
    * @param options is a property-like bottle containing the new 
    *                configuration employed by the internal pid.
    * @return true/false on success/failure. 
    *  
    * @note The property parameter should look like as follows: 
    * (Kp (1 2 ...)) (Ki (1 2 ...)) (Kd (1 2 ...)) (Wp (...)) ... 
    * @note The vectors dimension at pid creation time is always 
    *       retained.
    * @note The satLim property must be given ordered by rows. 
    * @note The sample time Ts should match the rate with which the 
    *       method @see lookAtStereoPixels is called by the user.
    * @note The special option "dominantEye" allows to select the 
    *       eye used for the monocular approach.
    */
    virtual bool setStereoOptions(const yarp::os::Bottle &options)=0;

    /** Bind the neck pitch within a specified range. [wait for
    *   reply]
    * @param min the minimum value of the range (in degree). 
    * @param max the maximum value of the range (in degree). 
    * @return true/false on success/failure. 
    */
    virtual bool bindNeckPitch(const double min, const double max)=0;

    /** Block the neck pitch at a specified angle. [wait for reply]
    * @param val the angle value at which block the joint (in 
    *           degree).
    * @return true/false on success/failure. 
    *  
    * @note The possibility to block the neck joints is given in 
    *       order to move just the eyes.
    */
    virtual bool blockNeckPitch(const double val)=0;

    /** Block the neck pitch at the current angle. [wait for reply]
    * @return true/false on success/failure. 
    *  
    * @note The possibility to block the neck joints is given in 
    *       order to move just the eyes. 
    */
    virtual bool blockNeckPitch()=0;

    /** Bind the neck roll within a specified range. [wait for
    *   reply]
    * @param min the minimum value of the range (in degree). 
    * @param max the maximum value of the range (in degree). 
    * @return true/false on success/failure. 
    */
    virtual bool bindNeckRoll(const double min, const double max)=0;

    /** Block the neck roll at a specified angle. [wait for reply]
    * @param val the angle value at which block the joint (in 
    *           degree).
    * @return true/false on success/failure. 
    *  
    * @note The possibility to block the neck joints is given in 
    *       order to move just the eyes.
    */
    virtual bool blockNeckRoll(const double val)=0;

    /** Block the neck roll at the current angle. [wait for reply]
    * @return true/false on success/failure. 
    *  
    * @note The possibility to block the neck joints is given in 
    *       order to move just the eyes. 
    */
    virtual bool blockNeckRoll()=0;

    /** Bind the neck yaw within a specified range. [wait for
    *   reply]
    * @param min the minimum value of the range (in degree). 
    * @param max the maximum value of the range (in degree). 
    * @return true/false on success/failure. 
    */
    virtual bool bindNeckYaw(const double min, const double max)=0;

    /** Block the neck yaw at a specified angle. [wait for reply]
    * @param val the angle value at which block the joint (in 
    *           degree).
    * @return true/false on success/failure. 
    *  
    * @note The possibility to block the neck joints is given in 
    *       order to move just the eyes.
    */
    virtual bool blockNeckYaw(const double val)=0;

    /** Block the neck yaw at the current angle. [wait for reply]
    * @return true/false on success/failure. 
    *  
    * @note The possibility to block the neck joints is given in 
    *       order to move just the eyes. 
    */
    virtual bool blockNeckYaw()=0;

    /** Return the current neck pitch range. [wait for reply]
    * @param min the location where to store the minimum of the 
    *            range [deg].
    * @param max the location where to store the maximum of the 
    *            range [deg].
    * @return true/false on success/failure. 
    */
    virtual bool getNeckPitchRange(double *min, double *max)=0;

    /** Return the current neck roll range. [wait for reply]
    * @param min the location where to store the minimum of the 
    *            range [deg].
    * @param max the location where to store the maximum of the 
    *            range [deg].
    * @return true/false on success/failure. 
    */
    virtual bool getNeckRollRange(double *min, double *max)=0;

    /** Return the current neck yaw range. [wait for reply]
    * @param min the location where to store the minimum of the 
    *            range [deg].
    * @param max the location where to store the maximum of the 
    *            range [deg].
    * @return true/false on success/failure. 
    */
    virtual bool getNeckYawRange(double *min, double *max)=0;

    /** Unblock the neck pitch. [wait for reply]
    * @return true/false on success/failure. 
    */
    virtual bool clearNeckPitch()=0;

    /** Unblock the neck roll. [wait for reply]
    * @return true/false on success/failure. 
    */
    virtual bool clearNeckRoll()=0;

    /** Unblock the neck yaw. [wait for reply]
    * @return true/false on success/failure. 
    */
    virtual bool clearNeckYaw()=0;

    /** Check once if the current trajectory is terminated. [wait for
    *   reply]
    * @param f where the result is returned.
    * @return true/false on success/failure.
    */
    virtual bool checkMotionDone(bool *f)=0;

    /** Wait until the current trajectory is terminated. [wait for
    *   reply]
    * @param period specify the check time period (seconds). 
    * @param timeout specify the check expiration time (seconds). If
    *         timeout<=0 (as by default) the check will be performed
    *         without time limitation.
    * @return true for success, false for failure and timeout 
    *         expired.
    */
    virtual bool waitMotionDone(const double period=0.1, const double timeout=0.0)=0;

    /** Ask for an immediate stop of the motion. [wait for reply]
    * @return true/false on success/failure. 
    */
    virtual bool stopControl()=0;

    /** Store the controller context. [wait for reply]
    * @param id specify where to store the returned context id. 
    * @return true/false on success/failure. 
    *  
    * @note The context comprises the values of internal controller 
    *       variables, such as the tracking mode, the trajectory
    *       time and so on.
    */
    virtual bool storeContext(int *id)=0;

    /** Restore the controller context previously stored. [wait for
    *   reply]
    * @param id specify the context id to be restored
    * @return true/false on success/failure. 
    *  
    * @note The context comprises the values of internal controller
    *       variables, such as the tracking mode, the trajectory
    *       time and so on.
    */
    virtual bool restoreContext(const int id)=0;
};

#endif


