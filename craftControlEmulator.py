#IMU Current Position Simulation:
#Given 6 IMU values, 3 for cartesian motion and 3 gyroscopes measuring orientation,
#provide the current position (Assuming IMUs are perfect and never fail to detect impulses)

#As this robot rotates, the IMU sensors contained within also rotate, throwing off the 
#Estimations for the current position and velocity of the unit. The goal of this program 
#is to provide a method for compensating for such deviations.

#This craft only has 3 engines and 3 internal flywheels capable of changing the orientation 
#of the craft. Using only positive thrust, the craft must autonomously steer towards 
#green zones
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import random

class robot():
    def __init__(self,scale=1):
        self.x=0
        self.y=0
        self.z=0
        self.vx=0
        self.vy=0
        self.vz=0
        self.roll=0     #About x
        self.pitch=0    #About y
        self.yaw=0      #About z
        self.cT=1/120   #Onboard Cycle Time is assumed to be 30hz
        self.time=time.time()
        self.polygonCount=9
        self.scale=scale/30
        self.color='blue'
        self.maxSpeed=scale/90
        self.maxAcceleration=scale/15
        self.maxTurnRate=30
    def processCycle(self,ax,ay,az,roll,pitch,yaw):
        deltaT=(time.time()-self.time)+self.cT
        self.time=time.time()
        ax,ay,az=map(float,np.array([ax,ay,az]) @ self.rotate3D(yaw,pitch,roll))
        if(ax>0):                           #enforce maximum acceleration values
            ax=min(ax,self.maxAcceleration)
        elif(ax<0):
            ax=max(ax,-self.maxAcceleration)
        if(ay>0):
            ay=min(ay,self.maxAcceleration)
        elif(ay<0):
            ay=max(ay,-self.maxAcceleration)
        if(az>0):
            az=min(az,self.maxAcceleration)
        elif(az<0):
            az=max(az,-self.maxAcceleration)
        if():
            self.vx=min(self.vx+deltaT*ax,self.maxSpeed)
        self.vy=min(self.vy+deltaT*ay,self.maxSpeed)
        self.vz=min(self.vz+deltaT*az,self.maxSpeed)
        self.x+=deltaT*self.vx
        self.y+=deltaT*self.vy
        self.z+=deltaT*self.vz
    def rotate3D(self,a,b,y):   #a=roll, b=pitch, y=yaw
        rollMat=     np.array([[math.cos(a),  -math.sin(a),    0],
                     [math.sin(a),   math.cos(a),     0],
                     [0,        0 ,         1]])
        pitchMat=   np.array([[math.cos(b),   0,      math.sin(b)],
                     [0,        1,      0],
                     [-math.sin(b),  0,      math.cos(b)]])
        yawMat=     np.array([[1,        0,      0],
                     [0,        math.cos(y), -math.sin(y)],
                     [0,        math.sin(y), math.cos(y)]])
        return yawMat @ pitchMat @ rollMat
    def orientYourselfNOW(self,x,y,z):  #x,y,z are the position of the target position
        #dxt=(x-self.x)/self.vx
        #dyt=(y-self.y)/self.vy
        #dzt=(z-self.z)/self.vz
        return(1,5,22,60,120,87)
    def plotOrb(self, ax):
        # Make orb shaped data
        u = np.linspace(0, 2 * np.pi, self.polygonCount)
        v = np.linspace(0, np.pi, self.polygonCount)
        # Plot the surface
        return ax.plot_surface(self.x + self.scale * np.outer(np.cos(u), np.sin(v)), self.y + self.scale * np.outer(np.sin(u), np.sin(v)), self.z + self.scale * np.outer(np.ones(np.size(u)), np.cos(v)), color=self.color)

class simulation():
    def __init__(self,simScale):
        self.ship=robot(simScale)
        self.simScale=simScale
    def run(self):
        print("ready for takeoff")
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        # Set the axis limits, so they aren't recalculated each frame.
        ax.set_xlim(0-self.simScale, self.simScale)
        ax.set_ylim(0-self.simScale, self.simScale)
        ax.set_zlim(0-self.simScale, self.simScale)

        # Begin plotting.
        ship=self.ship.plotOrb(ax)
        targetPos=(random.random()-0.5)*self.simScale*0.8,(random.random()-0.5)*self.simScale*0.8,(random.random()-0.5)*self.simScale*0.8
        for i in range(0,400):
            u = np.linspace(0, 2 * np.pi, 12)
            v = np.linspace(0, np.pi, 12)
            target = ax.plot_surface(targetPos[0] + (self.simScale/30) * np.outer(np.cos(u), np.sin(v)), targetPos[1] + (self.simScale/30) * np.outer(np.sin(u), np.sin(v)), targetPos[2] + (self.simScale/30) * np.outer(np.ones(np.size(u)), np.cos(v)), color='green')
            aX,ay,az,roll,pitch,yaw=self.ship.orientYourselfNOW(targetPos[0],targetPos[1],targetPos[2])
            self.ship.processCycle(aX,ay,az,roll,pitch,yaw)
            if ship:
                ship.remove()   #Removes the visualization of the object at the previous position.
            ship=self.ship.plotOrb(ax)
            plt.pause(0.03)

sim1=simulation(10000)
sim1.run()
