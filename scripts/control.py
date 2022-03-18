#!/usr/bin/env python3

#ROS Node which publishes waypoints to make the 3drobot perform squares.
import numpy as np
import rospy
import math
from std_msgs.msg import Float64
import sys
import os
import time

class control():
    def __init__(self):
        rospy.init_node('robot_puma_position', anonymous=True, argv=sys.argv)                      #Initialize ROS Node.
        coordenates = rospy.Subscriber('/coordenates',Float64, self.moveStraight2)

        self.joint1P = rospy.Publisher('/robot1/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2P = rospy.Publisher('/robot1/joint2_position_controller/command', Float64, queue_size=10) 
        self.joint3P = rospy.Publisher('/robot1/joint3_position_controller/command', Float64, queue_size=10) 
        self.joint4P = rospy.Publisher('/robot1/joint4_position_controller/command', Float64, queue_size=10) 
        self.joint5P = rospy.Publisher('/robot1/joint5_position_controller/command', Float64, queue_size=10) 
        self.joint6P = rospy.Publisher('/robot1/joint6_position_controller/command', Float64, queue_size=10)  

        
        #self.coor = rospy.Publisher('/coordenates', Float64, queue_size=10)  
        #self.coor.publish(1.0)
        
    def moveStraight2(self,data):
        Con = control()
        args = rospy.myargv(argv=sys.argv)
        Con.moveStraight(args)

    def moveStraight(self,data):

        #px = 0.4318
        #py = 0.1419
        #pz = 0.4321

        px = float(data[1])
        py = float(data[2])
        pz = float(data[3])

        print("px:")
        print(px)
        print("py:")
        print(py)
        print("pz:")
        print(pz)

        pi=math.pi
        aplha0 = 0
        aplha1 = -pi/2
        aplha2 = 0
        aplha3 = -pi/2
        aplha4 = pi/2
        aplha5 = -pi/2

        a0 = 0
        a1 = 0
        a2 = 0.43179
        a3 = 0
        a4 = 0
        a5 = 0

        d1 = 0
        d2 = 0
        d3 = 0.14185
        d4 = 0.43209
        d5 = 0
        d6 = 0

        tetha1 = math.atan2(py,px)-math.atan2(d3,math.sqrt(px**2+py**2-d3**2))
        k = (px**2+py**2+pz**2-a2**2-a3**2-d3**2-d4**2)/(2*a2)
        tetha3 = math.atan2(a3,d4)-math.atan2(k,-math.sqrt(a3**2+d4**2-k**2))

        c3 = math.cos(tetha3)
        s3 = math.sin(tetha3)
        c1 = math.cos(tetha1)
        s1 = math.sin(tetha1)

        tetha23 = math.atan2((-a3-a2*c3)*pz-(c1*px+s1*py)*(d4-a2*s3),(a2*s3-d4)*pz-(a3+a2*c3)*(c1*px+s1*py))
        tetha2 = tetha23-tetha3
        tetha4 = 0   
        tetha5 = 0 
        tetha6 = 0 

        print("Datos")

        tetha1=Float64(tetha1)
        tetha2=Float64(tetha2)
        tetha3=Float64(tetha3)


        print("angulos")
        print("tetha1")
        print(tetha1)
        print("tetha2")
        print(tetha2)
        print("tetha3")
        print(tetha3)
        print("tetha4")
        print(tetha4)
        print("tetha5")
        print(tetha5)
        print("tetha6")
        print(tetha6)

        self.joint1P.publish(tetha1)
        self.joint2P.publish(tetha2)
        self.joint3P.publish(tetha3)
        self.joint4P.publish(tetha4)
        self.joint5P.publish(tetha5)
        self.joint6P.publish(tetha6)
        
        
        #os.system('gnome-terminal -- bash -c "rostopic pub /coordenates std_msgs/Float64 1; exec bash"')
        time.sleep(1)

        print("MOVIMIENTO FINALIZADO")

if __name__ == '__main__': 

    Con = control()
    args = rospy.myargv(argv=sys.argv)

    if len(args) < 3:
        print("\nERROR: no file provided.")
        sys.exit()

    Con.moveStraight(args)
    rospy.spin()
    
