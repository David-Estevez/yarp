# Copyright: (C) 2009 RobotCub Consortium
# Author: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(microphone
                    CATEGORY device
                    TYPE yarp::dev::MicrophoneDeviceDriver
                    INCLUDE MicrophoneDeviceDriver.h
                    EXTRA_CONFIG WRAPPER=grabber
                    DEPENDS "CREATE_DEVICE_LIBRARY_MODULES;UNIX OR WIN32;NOT APPLE")

if(NOT SKIP_microphone)
  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  if(WIN32)
    set(_SYS Windows)
  else()
    set(_SYS Linux)
  endif()

  yarp_add_plugin(yarp_microphone ${_SYS}MicrophoneDeviceDriver.cpp
                                  MicrophoneDeviceDriver.h)
  target_link_libraries(yarp_microphone YARP::YARP_OS
                                        YARP::YARP_sig
                                        YARP::YARP_dev)
  yarp_install(TARGETS yarp_microphone
               EXPORT YARP
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})
  yarp_install(FILES microphone.ini
               COMPONENT runtime
               DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  set_property(TARGET yarp_microphone PROPERTY FOLDER "Deprecated")
endif()
