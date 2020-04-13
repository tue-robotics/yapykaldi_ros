from __future__ import (absolute_import, division, print_function, unicode_literals)
from hmi import AbstractHMIServer, HMIResult
from hmi.common import parse_sentence, verify_grammar
import os
import rospy
from std_msgs.msg import String
from yapykaldi.asr import Asr, WaveFileStreamer


class HMIServerYapykaldi(AbstractHMIServer):
    """HMI server wrapper yapykaldi ASR app"""
    def __init__(self):
        stream = WaveFileStreamer("/home/loy/output.wav")
        rospy.loginfo("Setting up ASR, may take a while...")
        self._asr = Asr(model_dir=os.path.expanduser(rospy.get_param('~model_dir')),
                        model_type=rospy.get_param('~model_type', 'nnet3'),
                        stream=stream)
        rospy.loginfo("Set up ASR")

        self._asr.register_fully_recognized_callback(self.string_fully_recognized_callback)
        self._asr.register_partially_recognized_callback(self.string_partially_recognized_callback)

        self._string = ""

        super(HMIServerYapykaldi, self).__init__(rospy.get_name())

    def stop(self):
        rospy.loginfo("Stopping {}".format(self))
        self._asr.stop()

    def string_fully_recognized_callback(self, string):
        # type: (str) -> None
        rospy.loginfo("Got a string: '{}'".format(string))
        self._string = string.strip()  # Using a threading primitive for this would be nicer!

    def string_partially_recognized_callback(self, string):
        # type: (str) -> None
        rospy.loginfo("Got an intermediate result string: '{}'".format(string))

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
        self._string = None

        verify_grammar(grammar)

        self._asr.start()

        while not rospy.is_shutdown() and not is_preempt_requested() and not self._string:
            rospy.logdebug("Sleep")
            rospy.sleep(.1)

        if rospy.is_shutdown() or is_preempt_requested():
            rospy.loginfo("Stop")
            self._asr.stop()
            return None

        rospy.loginfo("Received string: '%s'", self._string)

        semantics = parse_sentence(self._string, grammar, target)

        rospy.loginfo("Parsed semantics: %s", semantics)

        result = HMIResult(self._string, semantics)
        self._asr.stop()
        self._string = None

        return result
