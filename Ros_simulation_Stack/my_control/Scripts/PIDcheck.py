#!/usr/bin/env python
from __future__ import division
import rospy 
import time
import math
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

#Parameter Initialization
global a1,a2,a3
a1 = 34
a2 = 40
a3 = 32.6

global torque
torque = [0,0,0]

global accumulator_arr
accumulator_arr = [0,0,0]

global integral
integral = [0,0,0]

global pre_err
pre_err = [0,0,0]

global k1
k1 = [2,2,.001]

global k2
k2 = [5.5,5,.001]

global k3
k3 = [.1,1,.0001]
#------------------------------------------------------------------------------------------------
# Inverse Kinematics Function for Arm Joint Angles
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
#-------------------------------------------------------------------------------------------------
#Feedback Callback
def feedback(msg):
    feedbacks = msg.position    
    PID1(feedbacks[0])
    PID2(feedbacks[1])
    PID3(feedbacks[2])
    
#-------------------------------------------------------------------------------------------------
#Joint PID Controllers 
def PID1(angle):
    err = - angle + Arr[0]
    P = k1[0] * err
    der =((err - pre_err[0])/.1 ) 
    D = k1[1] * der
    integral[0] += + k1[2] * err * .1   
    I = integral[0]
    torque =  P + D + I
    pre_err[0] = err
    #Maximum Torque correction for the Joint
    if torque > 12:
        torque = 12
    elif torque < -12 :
        torque = -12
    else:
        pass
    #Unresponsive range Mapping
    if torque > 0 :
        torque = my_map(torque,0,12,2,12)
    else:
        torque = my_map(torque,-12,0,-12,-2)
    
    #unstabalization removal noise   
    if (torque > -2.02) & (torque < 2.02 ):
        torque = 0 
    j1 = rospy.Publisher('/iiwa/joint1_effort_controller/command', Float64, queue_size=10)
    j1.publish(torque)
    print(torque)
        
    
        
def PID2(angle):
    global test
    err = -angle + Arr[1] 
    P = k2[0] * err
    der =((err - pre_err[1])/.1 ) 
    D = k2[1] * der
    integral[1] += k2[2] * err * .1   
    I =  integral[1]
    torque =  P + D + I
    pre_err[1] = err
    
    if torque > 12:
        torque = 12
    elif torque < -12 :
        torque = -12
    else:
        pass
    
    if torque > 0 :
        torque = my_map(torque,0,12,2,15)
    else:
        torque = my_map(torque,-12,0,-15,-2)
    print(torque)
    if (torque > -2.4) & (torque < 2.4 ):
        torque = 0 
    j2 = rospy.Publisher('/iiwa/joint2_effort_controller/command', Float64, queue_size=10)
    j2.publish(torque)
    
        
def PID3(angle):
    
    err = Arr[2] - angle
    P = k2[0] * err
    der =((err - pre_err[2])/.1 ) 
    D = k2[1] * der
    integral[2] += + k2[2] * err * .1   
    I =  integral[2]
    torque =  P + D + I
    pre_err[2] = err
    
    if torque > 0:
        if  torque > 12:
            torque = 12
    else:
        if torque < -12:
            torque = -12
    print(torque)
    j3 = rospy.Publisher('/iiwa/joint4_effort_controller/command', Float64, queue_size=10)
    j3.publish(torque)
#-------------------------------------------------------------------------------------------------
#Mapping Function
def my_map(x,a,b,c,d):
    slope = (d-c)/(b-a) 
    y = ((x-a)*slope) + c
    return y
#-------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('PID', anonymous=True)
    global Arr #Angles target array
    rate = rospy.Rate(10)#rate in Hz for controller
    Arr = [0, 0, 0]    
    while not rospy.is_shutdown():
        sub_feedback = rospy.Subscriber("/iiwa/joint_states", JointState, feedback)
        rate.sleep()