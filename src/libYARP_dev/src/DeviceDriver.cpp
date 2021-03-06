/*
 * Copyright (C) 2006 RobotCub Consortium
 * Authors: Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <cstdio>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Value.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;



DeviceResponder::DeviceResponder() {
    makeUsage();
}

void DeviceResponder::addUsage(const char *txt, const char *explain) {
    examples.addString(txt); //Value::makeList(txt));
    explains.addString((explain!=NULL)?explain:"");
    details.add(Value::makeList(txt));
    ConstString more = ConstString("   ") + ((explain != YARP_NULLPTR) ? explain : "");
    details.addString(more.c_str());
}


void DeviceResponder::addUsage(const Bottle& bot, const char *explain) {
    addUsage(bot.toString().c_str(),explain);
}


bool DeviceResponder::respond(const Bottle& command, Bottle& reply) {
    switch (command.get(0).asVocab()) {
    case VOCAB4('h','e','l','p'):
        if (examples.size()>=1) {
            reply.add(Value::makeVocab("many"));
            if (command.get(1).toString()=="more") {
                reply.append(details);
            } else {
                reply.append(examples);
            }
            return true;
        } else {
            reply.addString("no documentation available");
            return false;
        }
        break;
    default:
        reply.addString("command not recognized");
        return false;
    }
    return false;
}

bool DeviceResponder::read(ConnectionReader& connection) {
    Bottle cmd, response;
    if (!cmd.read(connection)) { return false; }
    //printf("command received: %s\n", cmd.toString().c_str());
    respond(cmd,response);
    if (response.size()>=1) {
        ConnectionWriter *writer = connection.getWriter();
        if (writer!=NULL) {
            if (response.get(0).toString()=="many"&&writer->isTextMode()) {
                for (int i=1; i<response.size(); i++) {
                    Value& v = response.get(i);
                    if (v.isList()) {
                        v.asList()->write(*writer);
                    } else {
                        Bottle b;
                        b.add(v);
                        b.write(*writer);
                    }
                }
            } else {
                response.write(*writer);
            }

            //printf("response sent: %s\n", response.toString().c_str());
        }
    } else {
        ConnectionWriter *writer = connection.getWriter();
        if (writer!=NULL) {
            response.clear();
            response.addVocab(Vocab::encode("nak"));
            response.write(*writer);
        }
    }
    return true;
}


void DeviceResponder::makeUsage() {
    examples.clear();
    explains.clear();
    details.clear();
    addUsage("[help]", "list usage");
    addUsage("[help] [more]", "list usage with some comments");
}
