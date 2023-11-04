#!/usr/bin/env python
# encoding: utf8

import rospy
from rico_context.msg import HistoryEvent
import tiago_msgs.msg
from rico_context.srv import GetContext, GetContextResponse, ResetContext, ResetContextResponse, IsInTask, IsInTaskResponse, GetCurrentScenarioId, GetCurrentScenarioIdResponse

class RicoContext(object):
    def __init__(self):
        self.history = []
        self.current_task = None

        self.start_task_sub = rospy.Subscriber('/context/start_task', tiago_msgs.msg.Command, self.start_task_callback)
        self.sub = rospy.Subscriber('/context/push', HistoryEvent, self.push_callback)
        self.get_service = rospy.Service('/context/get', GetContext, self.get_context)
        self.reset_service = rospy.Service('/context/reset', ResetContext, self.reset_context)
        self.reset_scenario_service = rospy.Service('/context/reset_scenario', ResetContext, self.reset_scenario)
        self.is_in_task_service = rospy.Service('/context/is_in_task', IsInTask, self.is_in_task)
        self.get_current_scenario_id_service = rospy.Service('/context/scenario_id', GetCurrentScenarioId, self.get_current_scenario_id)
        self.get_context_after_last_scenario_service = rospy.Service('/context/get_after_last_scenario', GetContext, self.get_context_after_last_scenario)

    def start_task_callback(self, data):
        intent_name = data.intent_name
        params = {}
        for param_name, param_value in zip(data.param_names, data.param_values):
            params[param_name] = param_value

        self.current_task = {
            'intent_name': intent_name,
            'params': params
        }

        print("Current task: %s" % self.current_task)

    def push_callback(self, msg):
        is_idle = 'idle' in msg.complement

        previous_event = self.history[-1] if len(self.history) > 0 else None

        is_timeout_repeat = previous_event is not None and previous_event.actor == msg.actor and previous_event.action == msg.action and previous_event.complement == msg.complement

        if not is_idle and not is_timeout_repeat:
            self.history.append(msg)

        if msg.action == 'finish performing' and self.current_task is not None:
            self.current_task = None

        rospy.loginfo("History: %s", self.history)

    def get_context(self, req):
        rospy.loginfo("Get context: %s", req)
        return GetContextResponse(self.history)
    
    def get_context_after_last_scenario(self, req):
        rospy.loginfo("Get context after last scenario: %s", req)
        latest_history = self.history

        for i, event in enumerate(self.history):
            if event.actor == 'system' and event.action == 'finish scenario':
                latest_history = self.history[i+1:]

        return GetContextResponse(latest_history)
    
    def is_in_task(self, req):
        is_in_task = False

        for event in self.history:
            if event.actor == 'system' and event.action == 'trigger scenario':
                is_in_task = True
            elif event.actor == 'system' and event.action == 'finish scenario':
                is_in_task = False
            elif event.action == 'start performing' and 'idle' not in event.complement:
                is_in_task = True
            elif event.action == 'finish performing':
                is_in_task = False

        return IsInTaskResponse(is_in_task)
    
    def get_current_scenario_id(self, req):
        curr_scenario_id = None

        for event in self.history:
            if event.actor == 'system' and event.action == 'trigger scenario':
                curr_scenario_id = int(event.complement)

        return GetCurrentScenarioIdResponse(curr_scenario_id)
    
    def reset_context(self, req):
        rospy.loginfo("Reset context: %s", req)
        success = False

        try:
            self.history = []
            success = True
        except Exception as e:
            rospy.logerr("Error resetting context: %s", e)

        return ResetContextResponse(success)

    def reset_scenario(self, req):
        rospy.loginfo("Reset scenario: %s", req)
        new_history = []

        for i, event in enumerate(self.history):
            if event.actor == 'system' and event.action == 'trigger scenario':
                new_history = self.history[:i]

        self.history = new_history
            
        return ResetContextResponse(True)

def main():
    rospy.init_node('rico_context', anonymous=True)
    rospy.loginfo("rico_context node started")
    RicoContext()
    rospy.spin()

if __name__ == '__main__':
    main()
