#!/usr/bin/env python
import rospy 
import time
import math
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64 
global A
A_raw =[ 6.05670566824748  , -0.240216126655623 , 7.95352831520156  ,  62.6277646520052  ,  62.2929873264897   , 220.787467541030,
        -2.39947957734061   ,9.47502158584525   , 2.11340098081089  ,  -0.727520769827970  ,151.855404932387  ,  9.76018347666003,
        -7.58675254678478  , -3.18845764598821  , 5.68110232556983  ,  39.4828079786164   , 35.6493668237404  ,  222.696778420758]
A_raw = np.array(A_raw)
A = A_raw.reshape(3,6)



global a1,a2,a3
a1 = 34
a2 = 40
a3 = 52.6

global velocity
velocity = np.array([0, 0, 0]) 

def ik(x,y,z):
    r1 = math.sqrt(x**2 + y**2)
    r2 = a1 - z 
    if (r1 == 0) & (r2 >= 0):
        phi2 = math.pi/2
    elif (r1 ==0 ) & (r2 < 0):
        phi2 = -(math.pi/2)
    else:
        phi2 = math.atan(r2/r1)
    
    r3 =  math.sqrt(r1**2 + r2**2)
    
    if r3 ==0:
        phi1 = 0
    else :
        phi1 = math.acos((a3**2 - a2**2 - r3**2)/(-2*a2*r3))
        
    phi3 = math.acos((r3**2 - a2**2 -a3**2)/(-2*a2*a3))
    
    if y == 0:
        theta1 =0
    else:
        theta1 = math.atan(y/x)
    theta2 = phi2 - phi1
    theta3 = - math.pi + phi3
    Arr = np.array([theta1, theta2, theta3])
    return Arr
    
def feedback(msg):
    global feedbacks
    pos = np.array(msg.position)
    vel = np.array(msg.velocity)
    feedbacks = np.concatenate((pos,vel),axis= 0)
    feedbacks = feedbacks.reshape(1,6)
    com()

def com():
    global command   
    err = np.subtract(state,feedbacks)
    err = err.reshape(6,1)
    command = np.dot(A,err)
    command = command.reshape(1,3)
    
    command = np.divide(command,3)
    j1 = rospy.Publisher('/iiwa/joint1_effort_controller/command', Float64, queue_size=10)
    j2 = rospy.Publisher('/iiwa/joint2_effort_controller/command', Float64, queue_size=10)
    j3 = rospy.Publisher('/iiwa/joint4_effort_controller/command', Float64, queue_size=10)
    print(command)
    
    torque1 = command[0][0]
    torque2 = command[0][1]
    torque3 = command[0][2]
    j1.publish(torque1)
    j2.publish(torque2)
    j3.publish(torque3)

    
    
    
    
    
if __name__ == '__main__':
    rospy.init_node('LQR', anonymous=True)
    rate = rospy.Rate(50)
    x = 0
    y = 0
    z = 34+40+52.6    
    Arr = ik(x,y,z)
    global state
    state = np.concatenate((Arr,velocity),axis= 0)
    state = state.reshape(1,6)
    #creating state matrix complete
    
    while not rospy.is_shutdown():
        sub_feedback = rospy.Subscriber("/iiwa/joint_states", JointState, feedback)
        #for state feedback for joints
        rate.sleep()
        