from __future__ import (absolute_import, division, print_function, unicode_literals)
from hmi import AbstractHMIServer, HMIResult
import os
import rospy
from yapykaldi.asr import Asr


class HMIServerYapykaldi(AbstractHMIServer):
    """HMI server wrapper yapykaldi ASR app"""
    def __init__(self):
        super(HMIServerYapykaldi, self).__init__(rospy.get_name())

        self._asr = Asr(model_dir=os.path.expanduser(rospy.get_param('~model_dir')),
                        model_type=rospy.get_param('~model_type', 'nnet3'),
                        output_dir=rospy.get_param('~output_dir', '/tmp/hmi_server_yapykaldi'))

    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        """
        Method override to start speech recognition upon receiving a query
        from the HMI server

        :param description: (str) description of the HMI request
        :param grammar: (str) grammar that should be used
        :param target: (str) target that should be obtained from the grammar
        :param is_preempt_requested: (callable) checks whether a preempt is requested by the hmi client
        """
