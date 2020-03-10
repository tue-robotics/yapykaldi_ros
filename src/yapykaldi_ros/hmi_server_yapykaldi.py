from __future__ import (absolute_import, division, print_function, unicode_literals)
from builtins import *
import rospy
from yapykaldi import Asr
from hmi import AbstractHMIServer, HMIResult


class HMIServerYapykaldi(AbstractHMIServer):
    """HMI server wrapper yapykaldi ASR app"""
    def __init__(self):
        super().__init__(rospy.get_name())

    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        """
        Method override to start speech recognition upon receiving a query
        from the HMI server

        :param description: (str) description of the HMI request
        :param grammar: (str) grammar that should be used
        :param target: (str) target that should be obtained from the grammar
        :param is_preempt_requested: (callable) checks whether a preempt is requested by the hmi client
        """
