from __future__ import (absolute_import, division, print_function, unicode_literals)
from hmi import AbstractHMIServer, HMIResult
from hmi.common import parse_sentence, verify_grammar
import os
import rospy
from threading import Event
from yapykaldi.asr import Asr
from yapykaldi.io import WaveFileSource, PyAudioMicrophoneSource, WaveFileSink
from yapykaldi.logger import LOGGER_NAME as YAPYKALDI_LOGGER_NAME

from .rospy_logging import route_logger_to_ros


class HMIServerYapykaldi(AbstractHMIServer):
    """HMI server wrapper yapykaldi ASR app"""
    def __init__(self):

        # route_logger_to_ros(YAPYKALDI_LOGGER_NAME)
        # self.stream = WaveFileSource("/home/loy/output.wav")
        rospy.loginfo("Creating audio stream")
        self._source = PyAudioMicrophoneSource(saver=WaveFileSink("/tmp/recording.wav"))
        rospy.loginfo("Opening audio stream")
        self._source.open()

        rospy.loginfo("Setting up ASR, may take a while...")
        self._asr = Asr(model_dir=os.path.expanduser(rospy.get_param('~model_dir')),
                        model_type=rospy.get_param('~model_type', 'nnet3'),
                        stream=self._source)
        rospy.loginfo("Set up ASR")

        self._asr.register_callback(self.string_fully_recognized_callback)
        self._asr.register_callback(self.string_partially_recognized_callback, partial=True)

        self._partial_string = ""
        self._completed_string = ""
        self._speech_stopped = Event()

        self._voice_timer = None

        super(HMIServerYapykaldi, self).__init__(rospy.get_name())

    def stop(self):
        rospy.loginfo("Stopping {}".format(self))
        self._asr.stop()
        self._source.close()

    def string_fully_recognized_callback(self, string):
        # type: (str) -> None
        rospy.loginfo("Got a complete string: '{}'".format(string))
        self._completed_string = string.strip()  # Using a threading primitive for this would be nicer!

    def _voice_timer_elapsed(self, *args):
        if not self._completed_string:
            rospy.loginfo("Voice timer elapsed after not hearing something new in a while, "
                          "stopping ASR")
        else:
            rospy.logdebug("Voice timer elapsed after not hearing something new in a while, "
                           "stopping ASR")

        self._asr.stop()
        self._completed_string = self._partial_string
        if self._voice_timer:
            self._voice_timer.shutdown()

    def string_partially_recognized_callback(self, string):
        # type: (str) -> None
        # rospy.loginfo("Got an intermediate result string: '{}'".format(string))

        new = string.strip()
        same = new == self._partial_string
        if not same:
            rospy.loginfo("Heard something new: '{}'".format(new))
            if self._voice_timer:
                self._voice_timer.shutdown()
                self._voice_timer = None
                rospy.loginfo("Shutdown voice_timer")
        else:
            rospy.logdebug("This string is NOT new: '{}' == '{}'"
                .format(new, self._partial_string))
            if self._partial_string and not self._voice_timer:
                self._voice_timer = rospy.Timer(rospy.Duration(1),
                                                self._voice_timer_elapsed,
                                                oneshot=True)
                rospy.loginfo("Started voice_timer")
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
        if self._voice_timer:
            self._voice_timer.shutdown()
        self._voice_timer = None

        verify_grammar(grammar)

        self._asr.start()

        self._asr.recognize()

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
