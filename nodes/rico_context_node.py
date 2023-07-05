#!/usr/bin/env python
# encoding: utf8

import rospy
from rico_context.msg import HistoryEvent
from rico_context.srv import GetContext, GetContextResponse

class RicoContext(object):
    def __init__(self):
        self.history = []

        self.sub = rospy.Subscriber('/context/push', HistoryEvent, self.push_callback)
        self.get_service = rospy.Service('/context/get', GetContext, self.get_context)
    
    def push_callback(self, msg):
        self.history.append(msg)
        rospy.loginfo("History: %s", self.history)

    def get_context(self, req):
        rospy.loginfo("Get context: %s", req)
        return GetContextResponse(self.history)

def main():
    rospy.init_node('rico_context', anonymous=True)
    rospy.loginfo("rico_context node started")
    RicoContext()
    rospy.spin()

if __name__ == '__main__':
    main()