#!usr/bin/env python

#nodo tipo Teleop_key
from re import A
from pynput.keyboard import Key, Listener
#from getkey import getkey,keys
import rospy
from geometry_msgs.msg import Twist 
from turtlesim.srv import TeleportAbsolute,TeleportRelative
import argparse
import numpy as np
import termios,sys,os

#movimiento lineal
def pubVel(vel_x, ang_z, t):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('velPub', anonymous=False)
    vel = Twist()
    vel.linear.x = vel_x
    vel.angular.z = ang_z
    #rospy.loginfo(vel)
    endTime = rospy.Time.now() + rospy.Duration(t)
    while rospy.Time.now() < endTime:
        pub.publish(vel)

#movimiento absoluto (pose x,y,ang)
def teleport(x, y, ang):
    rospy.wait_for_service('/turtle1/teleport_absolute')
    try:
        teleportA = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        resp1 = teleportA(x, y, ang)
        #print('Teleported to x: {}, y: {}, ang: {}'.format(str(x),str(y),str(ang)))
    except rospy.ServiceException as e:
        print(str(e))

#movimiento relativo (linear, ang)
def move(x,ang):
    rospy.wait_for_service('/turtle1/teleport_relative')
    try:
        moveA = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)
        resp1 = moveA(x, ang)
        #print('Rotate x: {}, ang: {}'.format(str(x),str(ang)))
    except rospy.ServiceException as e:
        print(str(e))

def on_press(key):

    #teclas de movimiento
    if key == 'A' or 'a':
        #rotacion antihoraria,angz+
        pubVel(0,1,0.3)
    if key == 'd' or 'D':
        #rotacion horaria ,angz-
        pubVel(0,-1,0.3)
    if key == 'w' or 'w':
        #hacia adelante ,velx+
        pubVel(1,0,0.3)
    if key == 's' or 'S':
        #hacia atras, velx-
        pubVel(-1,0,0.3)
    #espacio para rotar 180°
    if key == Key.space:
        move(0,np.pi)
    #R para volver a posición y rotacion inicial
    if key == 'r'or'R':
        teleport(5.5,5.5,0)

def on_release(key):
    if key == Key.esc:
        return False

if __name__ == '__main__':
    pubVel(0,0,0.1)
    try:
        with Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    except rospy.ROSInterruptException:
        pass


 
