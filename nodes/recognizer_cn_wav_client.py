#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""
recognizer.py is a wrapper for pocketsphinx.
  parameters:
    ~lm - filename of language model
    ~dict - filename of dictionary
    ~mic_name - set the pulsesrc device name for the microphone input.
                e.g. a Logitech G35 Headset has the following device name: alsa_input.usb-Logitech_Logitech_G35_Headset-00-Headset_1.analog-mono
                To list audio device info on your machine, in a terminal type: pacmd list-sources
  publications:
    ~output (std_msgs/String) - text output
  services:
    ~start (std_srvs/Empty) - start speech recognition
    ~stop (std_srvs/Empty) - stop speech recognition
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy

import pygtk
pygtk.require('2.0')
import gtk

import gobject
import pygst
pygst.require('0.10')
gobject.threads_init()
import gst

from std_msgs.msg import String
from std_srvs.srv import *
import os
import commands
import time
import signal
import shutil
import datetime
is_exit = False

def handler(signum, frame):
    global is_exit
    is_exit = True
    print "receive a signal %d, is_exit = %d"%(signum, is_exit)

class recognizer(object):
    """ GStreamer based speech recognizer. """

    def __init__(self):
        # Start node
        rospy.init_node("recognizer_cn_client")

        self._device_name_param = "~mic_name"  # Find the name of your microphone by typing pacmd list-sources in the terminal

        # Configure mics with gstreamer launch config
        if rospy.has_param(self._device_name_param):
            self.device_name = rospy.get_param(self._device_name_param)
            self.device_index = self.pulse_index_from_name(self.device_name)
            self.launch_config = "pulsesrc device=" + str(self.device_index)
            rospy.loginfo("Using: pulsesrc device=%s name=%s", self.device_index, self.device_name)
        else:
            self.launch_config = 'gconfaudiosrc'
#            self.launch_config = 'filesrc location= /home/ppeix/music.wav ! wavparse '

        rospy.loginfo("Launch config: %s", self.launch_config)

        self.launch_config += " ! audioconvert ! audioresample " \
                            + "! tee name=t " \
                            + "! queue " \
                            + "! vader name=vad auto-threshold=true " \
                            + "! queue " \
                            + "! wavenc " \
                            + "! filesink location=/home/ppeix/music.wav "

#        rospy.loginfo("Launch config: %s", self.launch_config)

        self.launch_config_record="gconfaudiosrc ! audioconvert ! audioresample ! gdppay ! multifilesink next-file=4 max-file-size=1000000 location=/home/ppeix/wav/foo%05d.buff" 
        self.launch_config_convert="filesrc location=/home/ppeix/wav/$filename ! gdpdepay ! wavenc ! filesink location=/home/ppeix/wav/$filename_wav"

        # Configure ROS settings
        self.started = False
        
#        self.start_recognizer()
        self.start_record()
        self.start_convert()
#        rospy.on_shutdown(self.shutdown)
    def start_record(self):
        rospy.loginfo("Starting recorder... ")
        rospy.loginfo("Launch config: %s", self.launch_config_record)
        self.pipeline_record = gst.parse_launch(self.launch_config_record)
        self.pipeline_record.set_state(gst.STATE_PLAYING)
    def start_convert(self):
        global is_exit
        rospy.loginfo("Starting converting... ")
        loop = 0
        true1 = 1
        while true1:
            filename = 'foo%05d.buff'%loop
            filename_wav= 'foo%05d.wav'%(loop%5)
            #print filename_wav
            loop = loop +1
        #    if(loop==4):
         #       loop = 0

            while (os.path.isfile("/home/ppeix/wav/%s"%filename)!=True):
                time.sleep(0.01)
            while(True):
                filetime = os.path.getmtime("/home/ppeix/wav/%s"%filename)
                timenow = time.time()  
                timediff = (timenow-filetime)*1000 
                if(timediff > 100):
                    break
            print "file %s:%s exist!!timediff is %d ms!!"%(filename, filename_wav, timediff)
            self.launch_config_convert="filesrc location=/home/ppeix/wav/%s ! gdpdepay ! wavenc ! filesink location=/home/ppeix/wav/%s"%(filename, filename_wav)
            self.pipeline_convert = gst.parse_launch(self.launch_config_convert)
            self.pipeline_convert.set_state(gst.STATE_PLAYING)
            time.sleep(0.01)
            self.pipeline_convert.set_state(gst.STATE_NULL)
            os.system("rm /home/ppeix/wav/%s"%filename)
#            while(True):
#                if(self.pipeline_convert.get_state() == gst.STATE_PAUSED):
#                    break
#                if(is_exit == True):
#                    break;
            if (is_exit == True):
                break;
    def shutdown(self):
        gtk.main_quit()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    start = recognizer()
    gtk.main()

#                time.sleep(5)
#            fileName = os.path.join(DataPath, file)  
#            filemt = time.localtime(os.stat(filename).st_mtime)  
#            filetime = datetime.datetime( filemt[0] ,filemt[1] ,filemt[2], filemt[3])  
#            filetime = time.ctime(os.path.getmtime(filename))
#!/usr/bin/python 
# First write the buffer stream to .buff files (annotated using GStreamer's GDP format)
#gst-launch-0.10  -e filesrc location=/home/ppeix/music.wav ! wavparse ! gdppay ! multifilesink next-file=4 max-file-size=1000000 location=/home/ppeix/foo%05d.buff
# use the following instead for any other source (e.g. internet radio streams)
#gst-launch -e uridecodebin uri=http://url.to/stream ! gdppay ! multifilesink next-file=4 max-file-size=1000000 location=foo%05d.buff

# After we're done, convert each of the resulting files to proper .wav files with headers
#for file in *.buff; do
#    tgtFile="$(echo "$file"|sed 's/.buff$/.wav/')"
    
#        gst-launch-0.10 filesrc "location=$file" ! gdpdepay ! wavenc ! filesink "location=$tgtFile"
#    done
    
    # Uncomment the following line to remove the .buff files here, but to avoid accidentally 
    # deleting stuff we haven't properly converted if something went wrong, I'm not gonna do that now.
    #rm *.buff''"""""''"")"'')
#for file in *.buff; do
#    tgtFile="$(echo "$file"|sed 's/.buff$/.wav/')"
#    echo $tgtFile
#    gst-launch-0.10 -e filesrc "location=$file" ! gdpdepay ! wavenc ! filesink "location=$tgtFile" &
#done
    
    # Uncomment the following line to remove the .buff files here, but to avoid accidentally 
    # deleting stuff we haven't properly converted if something went wrong, I'm not gonna do that now.
    #rm *.buff''"""""''"")"'')
