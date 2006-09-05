// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef _YARP2_TESTFRAMEGRABBER_
#define _YARP2_TESTFRAMEGRABBER_

#include <stdio.h>

#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#define VOCAB_LINE VOCAB4('l','i','n','e')

namespace yarp {
    namespace dev {
        class TestFrameGrabber;
    }
}

/**
 * @ingroup dev_impl
 *
 * A fake camera for testing.
 * Implements the IFrameGrabberImage and IFrameGrabberControls
 * interfaces.
 */
class yarp::dev::TestFrameGrabber : public DeviceDriver, 
            public IFrameGrabberImage, public IFrameGrabberControls {
private:
    int ct;
    int bx, by;
    int w, h;
    unsigned long rnd;
    double period, freq;
    double first;
    double prev;
    int mode;

public:
    /**
     * Constructor.
     */
    TestFrameGrabber() {
        ct = 0;
        freq = 20;
        period = 1/freq;
        // just for nostalgia
        w = 128;
        h = 128;
        first = 0;
        prev = 0;
        rnd = 0;
    }


    virtual bool close() {
        return true;
    }

   /**
     * Configure with a set of options. These are:
     * <TABLE>
     * <TR><TD> width/w </TD><TD> Width of image (default 128). </TD></TR>
     * <TR><TD> height/h </TD><TD> Height of image (default 128). </TD></TR>
     * <TR><TD> freq </TD><TD> Frequency in Hz to generate images (default 20Hz). </TD></TR>
     * <TR><TD> period </TD><TD> Inverse of freq - only set one of these. </TD></TR>
     * <TR><TD> mode </TD><TD> Can be [ball] or [line] (default). </TD></TR>
     * </TABLE>
     *
     * @param config The options to use
     * @return true iff the object could be configured.
     */
    virtual bool open(yarp::os::Searchable& config) {
        yarp::os::Value *val;
        if (config.check("width",val)||config.check("w",val)) {
            w = val->asInt();
        }
        if (config.check("height",val)||config.check("h",val)) {
            h = val->asInt();
        }
        if (config.check("freq",val)) {
            freq = val->asDouble();
            period = 1/freq;
        }
        if (config.check("period",val)) {
            period = val->asDouble();
            freq = 1/period;
        }
        mode = config.check("mode",yarp::os::Value(VOCAB_LINE,
                                                   true)).asVocab();
        printf("Test grabber period %g / freq %g, mode [%s]\n", period, freq,
               yarp::os::Vocab::decode(mode).c_str());
        bx = w/2;
        by = h/2;
        return true;
    }

    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image) {

        double now = yarp::os::Time::now();

        if (now-prev>1000) {
            first = now;
            prev = now;
        }
        double dt = period-(now-prev);

        if (dt>0) {
            yarp::os::Time::delay(dt);
        }
        
        // this is the controlled instant when we consider the
        // image as going out
        prev += period;

        createTestImage(image);
        return true;
    }
    
    virtual int height() const {
        return h;
    }

    virtual int width() const {
        return w;
    }

    virtual bool setBrightness(double v) {
        return false;
    }

    virtual bool setShutter(double v) {
        return false;
    }

    virtual bool setGain(double v) {
        return false;
    }

    virtual double getBrightness() const {
        return 0;
    }

    virtual double getShutter() const {
        return 0;
    }

    virtual double getGain() const {
        return 0;
    }

private:
    void createTestImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);
};


/**
  * @ingroup dev_runtime
  * \defgroup cmd_device_test_grabber test_grabber

A fake framegrabber, see yarp::dev::TestFrameGrabber.

*/


#endif
