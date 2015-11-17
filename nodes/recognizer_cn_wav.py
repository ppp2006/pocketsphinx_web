#!/usr/bin/env python

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
is_exit =False
def handler(signum, frame):
    global is_exit
    is_exit = True
    print "receive a signal %d, is_exit = %d"%(signum, is_exit)
class recognizer(object):
    """ GStreamer based speech recognizer. """

    def __init__(self):
        # Start node
        rospy.init_node("recognizer_cn")

        self._device_name_param = "~mic_name"  # Find the name of your microphone by typing pacmd list-sources in the terminal
        self._lm_param = "~lm"
        self._dic_param = "~dict"

        # Configure mics with gstreamer launch config
        if rospy.has_param(self._device_name_param):
            self.device_name = rospy.get_param(self._device_name_param)
            self.device_index = self.pulse_index_from_name(self.device_name)
            self.launch_config = "pulsesrc device=" + str(self.device_index)
            rospy.loginfo("Using: pulsesrc device=%s name=%s", self.device_index, self.device_name)
        else:
            self.launch_config = 'filesrc location=/home/ppeix/wav/foo%05d.wav ! wavparse '

#        rospy.loginfo("Launch config: %s", self.launch_config)


        #rospy.loginfo("Launch config: %s", self.launch_config)
        # Configure ROS settings
        self.started = False
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('~output', String)
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)

        
        if rospy.has_param(self._lm_param) and rospy.has_param(self._dic_param):
            self.start_loop()
        else:
            rospy.logwarn("lm and dic parameters need to be set to start recognizer.")

    def start_recognizer(self):
        rospy.loginfo("Starting recognizer... ")
        rospy.loginfo("Launch config: %s", self.launch_config)
        self.pipeline = gst.parse_launch(self.launch_config)
        self.asr = self.pipeline.get_by_name('asr')
        self.asr.connect('partial_result', self.asr_partial_result)
        self.asr.connect('result', self.asr_result)
        self.asr.set_property('configured', True)
        self.asr.set_property('dsratio', 1)
        # Configure language model
        if rospy.has_param(self._lm_param):
            lm = rospy.get_param(self._lm_param)
        else:
            rospy.logerr('Recognizer not started. Please specify a language model file.')
            return

        if rospy.has_param(self._dic_param):
            dic = rospy.get_param(self._dic_param)
        else:
            rospy.logerr('Recognizer not started. Please specify a dictionary.')
            return

        self.asr.set_property('lm', lm)
        self.asr.set_property('dict', dic)

        self.bus = self.pipeline.get_bus()
        #self.bus.add_signal_watch()
        self.bus_id = self.bus.connect('message::application', self.application_message)
        self.pipeline.set_state(gst.STATE_PLAYING)
        self.started = True
    def start_loop(self):
        global is_exit
        rospy.loginfo("Starting recognizer loop... ")
        loop = 0
        true1 = 1
        while true1:
            filename = 'foo%05d.buff'%loop
            filename_wav= 'foo%05d.wav'%loop

            while (os.path.isfile("/home/ppeix/wav/%s"%filename_wav)!=True):
                time.sleep(0.01)
            while(True):
                filetime = os.path.getmtime("/home/ppeix/wav/%s"%filename_wav)
                timenow = time.time()  
                timediff = (timenow-filetime)*1000 
                if(timediff > 100):
                    break
            print "file %s exist!!timediff is %d ms!!"%(filename_wav, timediff)
            self.launch_config = "filesrc location= /home/ppeix/wav/foo%05d.wav ! wavparse "%loop
            self.launch_config += " ! audioconvert ! audioresample " \
                               + '! pocketsphinx hmm=tdt_sc_8kadapt lm=robot.lm.dmp dict=robot.dic name=asr ! fakesink'
            timelast = time.time()
            self.start_recognizer()
            rospy.loginfo("loop=%d recognizer begin:", loop)
            loop = loop +1
    #        if(loop == 4):
    #            loop = 0
            os.system("rm /home/ppeix/wav/%s"%filename_wav)
            os.system("rm /home/ppeix/wav/%s"%filename)
            while True:
                #time.sleep(1)
                #rospy.loginfo("sleeping...")
                if(self.started == False):
                    break
                if(is_exit == True):
                    break
                message = self.bus.poll(self.bus_id, 0)
                #self.application_message(self.bus_id, message)
                timenow=time.time()
                timediff=(timenow - timelast)*1000
                if(timediff >500):
                    rospy.loginfo("timediff is %d", timediff)
                    break
                #t=message.type
                #print t
                #print type(t)
            self.stop()
            if(is_exit == True):
                break

    def pulse_index_from_name(self, name):
        output = commands.getstatusoutput("pacmd list-sources | grep -B 1 'name: <" + name + ">' | grep -o -P '(?<=index: )[0-9]*'")

        if len(output) == 2:
            return output[1]
        else:
            raise Exception("Error. pulse index doesn't exist for name: " + name)

    def stop_recognizer(self):
        if self.started:
            self.pipeline.set_state(gst.STATE_NULL)
            self.pipeline.remove(self.asr)
            self.bus.disconnect(self.bus_id)
            
    def shutdown(self):
        """ Delete any remaining parameters so they don't affect next launch """
        for param in [self._device_name_param, self._lm_param, self._dic_param]:
            if rospy.has_param(param):
                rospy.delete_param(param)

        """ Shutdown the GTK thread. """
    #    gtk.main_quit()

    def start(self):
        self.start_recognizer()
        rospy.loginfo("recognizer started")
        return EmptyResponse()

    def stop(self):
        self.stop_recognizer()
        rospy.loginfo("recognizer stopped")
        return EmptyResponse()

    def asr_partial_result(self, asr, text, uttid):
        """ Forward partial result signals on the bus to the main thread. """
        struct = gst.Structure('partial_result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def asr_result(self, asr, text, uttid):
        """ Forward result signals on the bus to the main thread. """
        struct = gst.Structure('result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def application_message(self, bus, msg):
        """ Receive application messages from the bus. """
        rospy.loginfo("application_message from the bus received !!!")
        msgtype = msg.structure.get_name()
        if msgtype == 'partial_result':
            self.partial_result(msg.structure['hyp'], msg.structure['uttid'])
        if msgtype == 'result':
            self.final_result(msg.structure['hyp'], msg.structure['uttid'])

    def partial_result(self, hyp, uttid):
        """ Delete any previous selection, insert text and select it. """
        rospy.logdebug("Partial: " + hyp)
        rospy.loginfo("partial_result publish !!!")
    def final_result(self, hyp, uttid):
        """ Insert the final result. """
        rospy.loginfo("final_result publish !!!")
        msg = String()
        msg.data = str(hyp.lower())
        rospy.loginfo(msg.data)
        self.pub.publish(msg)
        self.started = False
if __name__ == "__main__":
    global finished
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    start=recognizer()
    gtk.main()

