import logging
import rospy


class RospyLogHandler(logging.Handler):
    def __init__(self, *args, **kwargs):
        super(RospyLogHandler, self).__init__(*args, **kwargs)

        self._mapping = {
            logging.INFO: rospy.loginfo,
            logging.DEBUG: rospy.logdebug,
            logging.WARN: rospy.logwarn,
            logging.ERROR: rospy.logerr,
        }

    def emit(self, record):
        # type (logging.LogRecord) -> None
        rospy_log = self._mapping[record.level]
        rospy_log(record.msg)
