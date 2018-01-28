'''Autonomous Agents from Nature of Code
January 26, 2017
p5-numpy version Jan 28'''

from flock import Flock
import numpy as np
import numpy.linalg as la
from p5 import *
from random import randint

class Boid:
    def __init__(self,x,y):
        self.location = np.array([x,y])
        self.velocity = np.array([float(randint(-2,2)),
                                float(randint(-2,2))])
        self.acceleration = np.array([0,0])
        self.maxspeed = 3.0
        self.maxforce = 0.05
        self.r = 5.0 #variable for size
        
    def run(self,boids):
        self.flock(boids)
        self.update()
        self.display()
        
    def update(self):
        #self.seek(PVector(mouseX,mouseY))
        #self.follow(Flowfield(50.0))
        self.velocity = np.add(self.velocity,self.acceleration)
        if la.norm(self.velocity) > self.maxspeed:
            self.velocity = np.divide(self.velocity,la.norm(self.velocity)) #set magnitude to 1
            self.velocity = np.multiply(self.velocity,self.maxspeed) #mult by parameter
        self.location += self.velocity
        if self.location[0] > width:
            self.location[0] = 0
        if self.location[1] > height:
            self.location[1] = 0
        if self.location[0] < 0: 
            self.location[0] = width
        if self.location[1] < 0:
            self.location[1] = height
        self.acceleration = np.multiply(self.acceleration,0)
        #self.display()
        
    def applyForce(self,force):
        self.acceleration = np.add(self.acceleration,force)
        
    def seek(self,target):
        self.desired = np.subtract(target,self.location)
        self.desired = np.divide(self.desired,la.norm(self.desired)) #normalize to 1
        self.desired = np.multiply(self.desired,self.maxspeed)
        #steer = np.array([0,0])
        steer = np.subtract(self.desired,self.velocity)
        #limit steer to maxforce
        if la.norm(steer) > self.maxforce:
            steer = np.divide(steer,la.norm(steer)) #set magnitude to 1
            steer = np.multiply(steer,self.maxforce) #mult by parameter
        #self.applyForce(steer)
        return steer
        
    def follow(self,flow):
        desired = flow.lookup(self.location)
        desired = np.multiply(desired,self.maxspeed)
        steer = np.subtract(desired,self.velocity)
        #limit steer to maxforce
        if la.norm(steer) > self.maxforce:
            steer = np.divide(steer,la.norm(steer)) #set magnitude to 1
            steer = np.multiply(steer,self.maxforce) #mult by parameter
        
        self.applyForce(steer)
        
    def separate(self,boidsList):
        desiredSeparation = self.r *2
        _sum = np.array([0,0])
        count = 0
        for boid in boidsList:
            d = dist(self.location,boid.location)
            if d > 0 and d < desiredSeparation:
                diff = np.subtract(self.location,boid.location)
                diff = np.divide(diff,la.norm(diff)) #normalize to 1
                diff = np.divide(diff,d)
                _sum = np.add(_sum,diff)
                count += 1
        if count > 0:
            _sum = np.divide(_sum,count)
        mag = la.norm(_sum)
        #print("mag:",mag)
        if mag > 0:
            _sum = np.divide(_sum, mag)
            _sum = np.multiply(_sum,self.maxspeed)
            _sum = np.divide(_sum,self.velocity)
            #limit _sum to maxforce
            if mag > self.maxforce:
                _sum = np.divide(_sum,mag) #set magnitude to 1
                _sum = np.multiply(_sum,self.maxforce) #mult by parameter
        
        return _sum
            
    def align(self,boids):
        neighbordist = 50
        _sum = np.array([0,0])
        count = 0
        for other in boids:
            d = dist(self.location,other.location)
            if d > 0 and d < neighbordist:
                _sum = np.add(_sum,other.velocity)
                count += 1
        if count > 0:
            _sum = np.divide(_sum,count)
            mag = la.norm(_sum)
            if mag > 0:
                _sum = np.divide(_sum,mag) #normalize, set mag to 1
                _sum = np.multiply(_sum,self.maxspeed)
            steer = np.subtract(_sum,self.velocity)
            #limit _sum to maxforce
            if mag > self.maxforce:
                _sum = np.divide(_sum,mag) #set magnitude to 1
                _sum = np.multiply(_sum,self.maxforce) #mult by maxforce
            return steer
        else:
            return np.array([0,0])
        
    def cohesion(self,boids):
        neighbordist = 50
        _sum = np.array([0,0])
        count = 0
        for other in boids:
            d = dist(self.location,other.location)
            if d > 0 and d < neighbordist:
                _sum = np.add(_sum,other.location)
                count += 1
        if count > 0:
            _sum = np.divide(_sum, count)
            return self.seek(_sum)
        else:
            return np.array([0,0])
            
    def flock(self,boidList):
        self.sep = self.separate(boidList)
        self.ali = self.align(boidList)
        self.coh = self.cohesion(boidList)
        
        #arbitrary weights. Try different ones!
        self.sep = np.multiply(self.sep,1.5)
        self.ali = np.multiply(self.ali,0.5)
        self.coh = np.multiply(self.coh,1.0)
        
        self.applyForce(self.sep)
        self.applyForce(self.ali)
        self.applyForce(self.coh)
        
    def display(self):
        theta = atan2(self.velocity[1],self.velocity[0]) + PI/2.0
        fill(175)
        stroke(0)
        push_matrix()
        translate(self.location[0],self.location[0])
        rotate(theta)
        #begin_shape()
        triangle([0,-self.r*2],[-self.r,self.r*2],
                 [self.r,self.r*2])
        #end_shape(CLOSE)
        reset_matrix()
        
        
def setup():
    global myflock
    size(600,600)
    #myboid = Boid(400,400)
    myflock = Flock()
    for i in range(10):
        myflock.addBoid(Boid(width/2,height/2))
    
def draw():
    background(255) #white
    myflock.run()
    '''for boid in myflock.boids:
        print(boid.acceleration)'''

run()
    
def mouse_dragged():
    myflock.addBoid(Boid(mouseX,mouseY))
