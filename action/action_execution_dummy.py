import rospy
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs

class ActionExecutionDummy():
    '''This is just a dummy to test the action dispatch method.
    It just sends 'action achieved' when receiving a task request.
    '''
    def __init__(self):
        rospy.init_node('Action_Receiver', anonymous=True)
        
        rospy.Subscriber('/kcl_rosplan/action_dispatch',
                        plan_dispatch_msgs.ActionDispatch,
                        self.send_action_feedback)

        self.action_feedback_pub = rospy.Publisher('/kcl_rosplan/action_feedback',
                        plan_dispatch_msgs.ActionFeedback,
                        queue_size=1,
                        latch = True)


    def send_action_feedback(self,msg):
        print("Received Action {}".format(msg.name))
        feedback_msg = plan_dispatch_msgs.ActionFeedback()

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'action_name'
        arg_msg.value = msg.name
        feedback_msg.information.append(arg_msg)
        
        feedback_msg.status = 'action achieved'

        self.action_feedback_pub.publish(feedback_msg)



if __name__ == '__main__':

    ActionExecutionDummy()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)