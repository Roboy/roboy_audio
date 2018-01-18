#!/usr/bin/env python

import rospy
import roboy_communication_cognition.msg
import message_filters

"""@package roboy_audio_stream
This module is a template for receiving the audio streams, sended by the roboy_audio/audio_capture module. 

"""

def stream_callback(data):
    """Just tells the world that it received data."""
    rospy.loginfo(rospy.get_caller_id() + 'I received data')


def stream_listener():
    """listens to the eight audio streams from the audio_common/capture_matrix_mics.launch module."""

    rospy.init_node('listener', anonymous=True)

    # one subscriber for every of the eight nodes
    for i in range(0, 8):
        rospy.Subscriber('/roboy/cognition/audio/mic' + str(i), roboy_communication_cognition.msg.AudioData, stream_callback)

    rospy.spin()


if __name__ == '__main__':
    stream_listener()
