#!/usr/bin/env python
"""
BlueRov video capture class
"""

import cv2
import gi
import os
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


class Video():

    def __init__(self, port=5601):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        os.environ['GST_DEBUG'] = '3'

        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None

        self.video_pipe = None
        self.video_sink = None

        self.loop = GLib.MainLoop()
        self.run()

    def start_gst(self):
        self.video_pipe = Gst.parse_launch(f"udpsrc do-timestamp=true port={self.port} ! application/x-rtp ! rtph264depay ! avdec_h264 discard-corrupted-frames=true ! videoconvert ! video/x-raw,format=BGR ! appsink name=appsink0 sync=false emit-signals=true")
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8
        )
        return cv2.cvtColor(array, cv2.COLOR_BGR2RGB)

    def frame(self):
        """ Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """ Get frame to update _new_frame
        """

        self.start_gst()

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK

if __name__ == '__main__':
    # Create the video object
    # Add port= if is necessary to use a different one
    video = Video(5601)

    print('Initialising stream...')
    waited = 0
    while not video.frame_available():
        waited += 1
        print('\r  Frame not available (x{})'.format(waited), end='')
        cv2.waitKey(30)
    print('\nSuccess!\nStarting streaming - press "q" to quit.')

    while True:
        # Wait for the next frame to become available
        if video.frame_available():
            # Only retrieve and display a frame if it's new
            frame = video.frame()
            cv2.imshow('frame', frame)
        # Allow frame to display, and check if user wants to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
