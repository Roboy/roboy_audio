#!/usr/bin/env python

import rospy
import audio_common_msgs.msg
import numpy as np
import array
import sounddevice as sd

import pygst
pygst.require("0.10")
import gst
import pygtk
import gtk
from threading import Thread
import glib
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import GObject

class RosGstPlay():


    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    def __init__(self):
        rospy.init_node('stream_listener', anonymous=True)

        self._sub = rospy.Subscriber('/mic0/audio', audio_common_msgs.msg.AudioData, self.onAudio(), queue_size=10)

        self.gst_loop = glib.MainLoop.__new__(None, False)

        # trying to copy the gstreamer stuff in a new way
        self.gst_pipeline = gst.Pipeline("app_pipeline")
        self.gst_source = gst.element_factory_make("appsrc", "app_source")
        gst.Bin.add(self.gst_pipeline, self.gst_source)

        # g_signal_connect(_source, "need-data", G_CALLBACK(cb_need_data),this);

        self.gst_decoder = gst.element_factory_make("decodebin", "decoder")
        # g_signal_connect(_decoder, "pad-added", G_CALLBACK(cb_newpad),this);
        gst.Bin.add(self.gst_pipeline, self.gst_decoder)
        gst.Element.link(self.gst_source, self.gst_decoder)

        self.gst_audio = gst.Bin.new("audiobin")
        self.gst_conver = gst.element_factory_make("audioconvert", "convert")
        audiopad = gst.Element.get_static_pad(self.gst_conver, "sink")
        self.gst_sink = gst.element_factory_make("autoaudiosink", "sink")
        # since add many is not available
        gst.Bin.add(self.gst_audio, self.gst_conver)
        gst.Bin.add(self.gst_audio, gst_sink)
        gst.Bin.add(self.gst_audio, None)
        gst.Element.link(self.gst_conver, gst_sink)
        gst.Element.add_pad(self.gst_audio, gst.GhostPad.new("sink", audiopad))
        gst.Object.unref(audiopad)

        gst.Bin.add(self.gst_pipeline, self.gst_audio)

        gst.Element.set_state(self.gst_pipeline, gst.State.PLAYING)

        self.gst_thread = Thread(glib.MainLoop.run, self.gst_loop)

        self.gst_paused = False

    def onAudio(self, data):
        if self.gst_paused:
            gst.Element.set_state(self.gst_pipeline, gst.State.PLAYING)
            self.gst_paused = False

        gst.Buffer.new_allocate()


if __name__ == '__main__':
    stream_listener()
