#!/usr/bin/env python
import rospy 
import time
import math
from geometry_msgs.msg import Vector3 
global a1,a2,a3
a1 = 34
a2 = 40
a3 = 52.6

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
    Arr = [theta1, theta2, theta3]
    return Arr
    


if __name__ == '__main__':
    rospy.init_node('ik_Coverter', anonymous=False)
    x = 0
    y = 0
    z = 34+40+52.6
    Theta = Vector3()
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/iiwa/theta', Vector3, queue_size=10)
    while not rospy.is_shutdown():
        Arr = ik(x,y,z)
        Theta.x = Arr[0]
        Theta.y = Arr[1]
        Theta.z = Arr[2]
        pub.publish(Theta)
        print(Theta)
        rate.sleep()

    