import rospy
import time
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
#from action.action import Action

# Action Topic: 'action_dispatch_topic'
#ROSPlan 'ActionDispatch.msg'
##actionDispatch message
#int32 action_id
#string name
#diagnostic_msgs/KeyValue[] parameters
#float32 duration
#float32 dispatch_time

#name --> Action Name
#paramters --> diag_msgs.KeyValue() pairs for parameters
#others not used

# Feedback Topic: 'action_feedback_topic'
#ROSPlan 'ActionFeedback.msg'
##actionFeedback message
#int32 action_id
#string status
#diagnostic_msgs/KeyValue[] information

#status 'action achieved' or 'action failed'
#information['action_name'] --> Action Name

class ActionDispatcher:
    '''A very simple ROSplan action dispatcher
    '''
    def __init__(self):

        #init ROS node
        rospy.init_node("Action_Dispatcher",anonymous = True)

        #create publischer for action dispatcher
        self.action_dispatch_pub = rospy.Publisher('/kcl_rosplan/action_dispatch',
                                                   plan_dispatch_msgs.ActionDispatch,
                                                   queue_size=1,
                                                   latch = True)
        #subsribe to feedback topic
        rospy.Subscriber('/kcl_rosplan/action_feedback',
                         plan_dispatch_msgs.ActionFeedback,
                         self._get_action_feedback)

        self.action_name = ''
        self.executing = False
        self.succeeded = False

    def _get_action_feedback(self, msg):
        if msg.information and msg.information[0].key == 'action_name' and \
        msg.information[0].value == self.action_name:
            self.executing = False
            self.succeeded = (msg.status == 'action achieved')
    
    def dispatch_action(self, action, timeout=10):
        '''
        Dispatch Action <action>. 
        Wait for success for <timeout> seconds.
        Return 'succeeded' or 'failed'
        '''

        #create dispatch message
        dispatch_msg = plan_dispatch_msgs.ActionDispatch()

        self.action_name = action.type
        dispatch_msg.name = action.type

        for key in action.parameters:
            arg_msg = diag_msgs.KeyValue()
            arg_msg.key = key
            arg_msg.value = action.parameters[key]
            dispatch_msg.parameters.append(arg_msg)

        #publish action message and wait for feedback
        print("Dispatching Action {}".format(action.type))
        self.action_dispatch_pub.publish(dispatch_msg)
        self.executing = True
        self.succeeded = False
        start_time = time.time()
        duration = 0.
        while self.executing and duration < timeout:
            rospy.sleep(0.1)
            duration = time.time() - start_time

        if self.succeeded:
            return 'succeeded'
        else:
            return 'failed'