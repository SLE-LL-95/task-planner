import rospy
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs

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

class action_dipatcher:
    def __init__(self):

        #create publischer for action dispatcher
        self.action_dispatch_pub = rospy.Publisher('/kcl_rosplan/action_dispatch',
                                                   plan_dispatch_msgs.ActionDispatch,
                                                   queue_size=1)

        #subsribe to feedback topic
        rospy.Subscriber('/kcl_rosplan/action_feedback',
                         plan_dispatch_msgs.ActionFeedback,
                         self.get_action_feedback)
        
        self.action_name = ''
        self.executing = False
        self.succeeded = False

    def get_action_feedback(self, msg):
        if msg.information and msg.information[0].key == 'action_name' and \
        msg.information[0].value == self.action_name:
            self.executing = False
            self.succeeded = (msg.status == 'action achieved')
    
    def dispatch_action(self, action):
        dispatch_msg = plan_dispatch_msgs.ActionDispatch()
        dispatch_msg.name = action.type

        
        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'bot'
        arg_msg.value = self.robot_name
        dispatch_msg.parameters.append(arg_msg)