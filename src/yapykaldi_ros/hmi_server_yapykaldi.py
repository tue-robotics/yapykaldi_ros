from __future__ import (absolute_import, division, print_function, unicode_literals)
from hmi import AbstractHMIServer, HMIResult
from hmi.common import parse_sentence, verify_grammar
import logging
import os
import rospy
from threading import Event
from yapykaldi.asr import Asr
from yapykaldi.audio_handling.sources import WaveFileSource, PyAudioMicrophoneSource
from yapykaldi.audio_handling.sinks import WaveFileSink

from .rospy_logging import RospyLogHandler

rpl = RospyLogHandler()
rpl.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
root = logging.getLogger()
root.setLevel(logging.DEBUG)
root.addHandler(rpl)


class HMIServerYapykaldi(AbstractHMIServer):
    """HMI server wrapper yapykaldi ASR app"""
    def __init__(self):

        self.stream = WaveFileSource("/home/loy/output.wav")
        # self.stream = PyAudioMicrophoneSource(saver=WaveFileSink("/tmp/recording.wav"))
        rospy.loginfo("Opening audio stream")
        self.stream.open()
        rospy.loginfo("Setting up ASR, may take a while...")
        self._asr = Asr(model_dir=os.path.expanduser(rospy.get_param('~model_dir')),
                        model_type=rospy.get_param('~model_type', 'nnet3'),
                        stream=self.stream)
        rospy.loginfo("Set up ASR")

        self._asr.register_fully_recognized_callback(self.string_fully_recognized_callback)
        self._asr.register_partially_recognized_callback(self.string_partially_recognized_callback)

        self._partial_string = ""
        self._completed_string = ""
        self._speech_stopped = Event()

        self.timeout_timer = None

        super(HMIServerYapykaldi, self).__init__(rospy.get_name())

    def stop(self):
        rospy.loginfo("Stopping {}".format(self))
        self._asr.stop()

    def string_fully_recognized_callback(self, string):
        # type: (str) -> None
        rospy.loginfo("Got a string: '{}'".format(string))
        self._completed_string = string.strip()  # Using a threading primitive for this would be nicer!

    def _voice_timer_elapsed(self, *args):
        if not self._completed_string:
            rospy.loginfo("Voice timer elapsed after not hearing something ew in a while")
        else:
            rospy.logdebug("Voice timer elapsed after not hearing something ew in a while")

        self._asr.stop()
        self._completed_string = self._partial_string
        self.timeout_timer.shutdown()

    def string_partially_recognized_callback(self, string):
        # type: (str) -> None
        # rospy.loginfo("Got an intermediate result string: '{}'".format(string))

        new = string.strip()
        same = new == self._partial_string
        if not same:
            rospy.loginfo("Heard something new: '{}'".format(new))
            if self.timeout_timer:
                self.timeout_timer.shutdown()

        else:
            rospy.logdebug("This string is NOT new: '{}' == '{}'"
                .format(new, self._partial_string))
            if self._partial_string:
                self.timeout_timer = rospy.Timer(rospy.Duration(3),
                                                 self._voice_timer_elapsed,
                                                 oneshot=True)
            # self._voice_timer_elapsed(None)
        self._partial_string = new

    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        # type: (str, str, str, Func) -> None
        """
        Method override to start speech recognition upon receiving a query
        from the HMI server

        :param description: (str) description of the HMI request
        :param grammar: (str) grammar that should be used
        :param target: (str) target that should be obtained from the grammar
        :param is_preempt_requested: (callable) checks whether a preempt is requested by the hmi client
        """
        rospy.loginfo("Need an answer from ASR for '{}'".format(description))
        self._completed_string = None
        self._speech_stopped.clear()
        if self.timeout_timer:
            self.timeout_timer.shutdown()
        self.timeout_timer = None

        verify_grammar(grammar)

        self._asr.start()

        while not rospy.is_shutdown() and \
                not is_preempt_requested() and \
                not self._completed_string and \
                not self._speech_stopped.is_set():
            rospy.logdebug("Sleep")
            rospy.sleep(.1)

        if rospy.is_shutdown() or is_preempt_requested():
            rospy.loginfo("Stop")
            self._asr.stop()
            return None

        rospy.loginfo("Received string: '%s'", self._completed_string)

        semantics = parse_sentence(self._completed_string, grammar, target)

        rospy.loginfo("Parsed semantics: %s", semantics)

        result = HMIResult(self._completed_string, semantics)
        self._asr.stop()
        self._completed_string = None

        return result
