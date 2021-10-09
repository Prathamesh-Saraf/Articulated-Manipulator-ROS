#!/usr/bin/env python
import rospy 
import time
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

def feedback(msg):
    feedback = msg.position
    r1 = rospy.Publisher('/iiwa/joint1_effort_controller/response1', Float64, queue_size=10)
    r2 = rospy.Publisher('/iiwa/joint1_effort_controller/response2', Float64, queue_size=10)
    r3 = rospy.Publisher('/iiwa/joint1_effort_controller/response3', Float64, queue_size=10)
    
    r1.publish(feedback[0])
    r2.publish(feedback[1])
    r3.publish(feedback[2])



if __name__ == '__main__':
    rospy.init_node('respose', anonymous=True)
    print(12/40)
    sub_feedback = rospy.Subscriber("/iiwa/joint_states", JointState, feedback)
        #for state feedback for joints
    rospy.spin()