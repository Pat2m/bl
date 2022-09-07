# -*- coding: utf-8 -*-
"""
Created on Tue Aug 23 17:07:23 2022

@author: Pat
"""
import numpy as np
from scipy.linalg import norm
from matplotlib import pyplot as plt
from Quaternion import Quaternion
import socket
import json

class Limb():
    def __init__(self, name):
        self.length = 1
        self.start = (0, 0, 0)
        self.end = (0, 0, 0)
        self.orientation = None
        self.name = name


def dump_json(data):
    data = json.loads(data)
    limb = data["Limb"]
    data = data["data"]
    match limb:
        case "left_hand":
            left_hand.orientation = data["orientation"]
            left_hand.start = data["start"]
            left_hand.end = data["end"]
        case "left_forearm":
            left_forearm.orientation = data["orientation"]
            left_forearm.start = data["start"]
            left_forearm.end = data["end"]
        case "left_bicep":
            left_bicep.orientation = data["orientation"]
            left_bicep.start = data["start"]
            left_bicep.end = data["end"]
        case "right_hand":
            right_hand.orientation = data["orientation"]
            right_hand.start = data["start"]
            right_hand.end = data["end"]
        case "right_forearm":
            right_forearm.orientation = data["orientation"]
            right_forearm.start = data["start"]
            right_forearm.end = data["end"]
        case "right_bicep":
            right_bicep.orientation = data["orientation"]
            right_bicep.start = data["start"]
            right_bicep.end = data["end"]
        case "left_foot":
            left_foot.orientation = data["orientation"]
            left_foot.start = data["start"]
            left_foot.end = data["end"]
        case "left_shin":
            left_shin.orientation = data["orientation"]
            left_shin.start = data["start"]
            left_shin.end = data["end"] 
        case "left_thigh":
            left_thigh.orientation = data["orientation"]
            left_thigh.start = data["start"]
            left_thigh.end = data["end"]
        case"right_foot":
            right_foot.orientation = data["orientation"]
            right_foot.start = data["start"]
            right_foot.end = data["end"]
        case"right_shin":
            right_shin.orientation = data["orientation"]
            right_shin.start = data["start"]
            right_shin.end = data["end"]
        case"right_thigh":
            right_thigh.orientation = data["orientation"]
            right_thigh.start = data["start"]
            right_thigh.end = data["end"]
        case"upper_torso":
            upper_torso.orientation = data["orientation"]
            upper_torso.start = data["start"]
            upper_torso.end = data["end"]
        case"lower_torso":
            lower_torso.orientation = data["orientation"]
            lower_torso.start = data["start"]
            lower_torso.end = data["end"]
        case"head":
            head.orientation = data["orientation"]
            head.start = data["start"]
            head.end = data["end"]
    

def drawCir(point,n,scale):
    x, y, z = point
    u = np.linspace(0, 2*np.pi, n)
    v = np.linspace(0, np.pi, n)
    x =  scale * np.outer(np.cos(u), np.sin(v)) + x
    y =  scale * np.outer(np.sin(u), np.sin(v)) + y
    z =  scale * np.outer(np.ones(np.size(u)), np.cos(v)) + z
    return x,y,z

def drawCyl(p0, v, mag, R):
    v = v.to_unit_vector()
    x, y, z = v
    v = np.array([x, y, z])
    #make some vector not in the same direction as v
    not_v = np.array([1, 0, 0])
    if (v == not_v).all():
            not_v = np.array([0, 1, 0])
    
    #make vector perpendicular to v
    n1 = np.cross(v, not_v)
    #normalize n1
    if norm(n1) != 0:
            n1 /= norm(n1)
            #make unit vector perpendicular to v and n1
            n2 = np.cross(v, n1)
    else:
        n1 = [0,1,0]
        n2 = np.cross(v, n1)
        #print("exception")
    
    #surface ranges over t from 0 to length of axis and 0 to 2*pi
    t = np.linspace(0, mag, 2)
    theta = np.linspace(0, 2 * np.pi, 100)
    rsample = np.linspace(0, R, 2)
    
    #use meshgrid to make 2d arrays
    t, theta2 = np.meshgrid(t, theta)
    
    rsample,theta = np.meshgrid(rsample, theta)
    x, y, z = p0
    p0 = np.array([x, y, z]) 
    print(n1)
    print(n2)
    #generate coordinates for surface
    # "Tube"
    X, Y, Z = [p0[i] + v[i] * t + R * np.sin(theta2) * n1[i] + R * np.cos(theta2) *       n2[i] for i in [0, 1, 2]]
    # "Bottom"
    X2, Y2, Z2 = [p0[i] + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    # "Top"
    X3, Y3, Z3 = [p0[i] + v[i]*mag + rsample[i] * np.sin(theta) * n1[i] + rsample[i] * np.cos(theta) * n2[i] for i in [0, 1, 2]]
    return [X,Y,Z], [X2,Y2,Z2], [X3,Y3,Z3]

def plot_skeleton(ax):

    for i in range(len(limbs)):
        x, y, z = drawCir(limbs[i].start, 100, 1)
        ax.plot_surface(x, y, z,rstride=4, cstride=4, color="yellow")
        v = limbs[i].orientation.to_unit_vector()
        s1,t1,b1 = drawCyl(limbs[i].start, v, limbs[i].length, .5)
        ax.plot_surface(s1[0], s1[1], s1[2], color='green')
        ax.plot_surface(t1[0], t1[1], t1[2], color='green')
        ax.plot_surface(b1[0], b1[1], b1[2], color='green')
        '''
    x, y, z = drawCir(Skeleton.get_end_position(Skeleton.head_pos, Skeleton.head, Skeleton.head_length),
                       100, 2)
    ax.plot_surface(x, y, z,rstride=4, cstride=4, color="yellow")
    '''
    return ax




def main():
    # Limbs need to be accessed by sensor handler.
    global left_hand, left_forearm, left_bicep
    global right_hand, right_forearm, right_bicep
    global left_foot, left_shin, left_thigh
    global right_foot, right_shin, right_thigh
    global upper_torso, lower_torso, head, s
    global shoulder_width, hip_width, limbs
    limbs = [
    left_hand = Limb("left_hand"),
    left_forearm = Limb("left_forearm"),
    left_bicep = Limb("left_bicep"),
    right_hand = Limb("right_hand"),
    right_forearm = Limb("right_forearm"),
    right_bicep = Limb("right_bicep"),
    left_foot = Limb("left_foot"),
    left_shin = Limb("left_shin"),
    left_thigh = Limb("left_thigh"),
    right_foot = Limb("right_foot"),
    right_shin = Limb("right_shin"),
    right_thigh = Limb("right_thigh"),
    upper_torso = Limb("upper_torso"),
    lower_torso = Limb("lower_torso"),
    head = Limb("head"),
    ]
    # Set user length. Initialize Limbs.
    d = input("Hand length: ")
    left_hand.length = int(d)
    right_hand.length = int(d)
    print(left_hand.factor)
    d = input("Forearm length: ")
    left_forearm.length = int(d) 
    right_forearm.length = int(d)
    d = input("Bicep length: ")
    left_bicep.length = int(d)
    right_bicep.length = int(d)
    d = input("Torso length: ")
    upper_torso.length = int(d) / 2
    lower_torso.length = int(d) / 2
    d = input("Thigh length: ")
    left_thigh.length = int(d)
    right_thigh.length = int(d)
    d = input("Shin length: ")
    left_shin.length = int(d)
    right_shin.length = int(d)
    d = input("Foot length: ")
    left_foot.length = int(d)
    right_foot.length = int(d)
    d = input("Head length: ")
    head.length = int(d)

    # Set user width.
    shoulder_width = int(input("Shoulder width: "))
    hip_width = int(input("Hip width: "))

    # Calibrate
    input("Stand in a T-pose. Press enter to calibrate.")

    # Set initial orientation.
    left_hand.orientation = Quaternion(0, 0, .7071, .7071)
    left_forearm.orientation = Quaternion(0, 0, .7071, .7071)
    left_bicep.orientation = Quaternion(0, 0, .7071, .7071)
    left_thigh.orientation = Quaternion(0, .7071, .7071, 0)
    left_shin.orientation = Quaternion(0, .7071, .7071, 0)
    left_foot.orientation = Quaternion(0, .7071, 0, .7071)
    right_hand.orientation = Quaternion(0, 0, .7071, -.7071)
    right_forearm.orientation = Quaternion(0, 0, .7071, -.7071)
    right_bicep.orientation = Quaternion(0, 0, .7071, -.7071)
    right_thigh.orientation = Quaternion(0, .7071, .7071, 0)
    right_shin.orientation = Quaternion(0, .7071, .7071, 0)
    right_foot.orientation = Quaternion(0, .7071, 0, .7071)
    head.orientation = Quaternion()
    upper_torso.orientation = Quaternion()
    lower_torso.orientation = Quaternion()
    
    plt.ion()
    fig = plt.figure()
    ax=plt.axes(projection='3d')
    ax.axes.set_xlim3d(left=-15, right=15) 
    ax.axes.set_ylim3d(bottom=-15, top=15) 
    ax.axes.set_zlim3d(bottom=0, top=30) 
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    fig.canvas.draw()

    serverMACAddress = 'A4:B1:C1:33:DB:27'
    port = 3
    backlog = 1
    size = 1024
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.bind((hostMACAddress,port))
    s.listen(backlog)
    print("opening socket")
    try:
        client, address = s.accept()
        while 1:
            data = client.recv(size)
            if data:
                fig.canvas.flush_events()
                print(data)
                dump_json(data)
                ax = plot_skeleton(ax)
                fig.canvas.draw()
                client.send(data)
    except:	
        print("Closing socket")	
        client.close()
        s.close()
    


if __name__ == "__main__":
    main()



