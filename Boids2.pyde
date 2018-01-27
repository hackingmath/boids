'''Autonomous Agents from Nature of Code
January 26, 2017
with Curtis'''

from flowfield import Flowfield
from flock import Flock

class Boid:
    def __init__(self,x,y):
        self.location = PVector(x,y)
        self.velocity = PVector(0,0)
        self.acceleration = PVector(0,0)
        self.maxspeed = 4
        self.maxforce = 0.1
        self.r = 3.0 #variable for size
        
    def run(self,boids):
        self.flock(boids)
        self.update()
        self.display()
        
    def update(self):
        #self.seek(PVector(mouseX,mouseY))
        #self.follow(Flowfield(50.0))
        self.velocity.add(self.acceleration)
        self.velocity.limit(self.maxspeed)
        self.location.add(self.velocity)
        if self.location.x > width:
            self.location.x = 0
        if self.location.y > height:
            self.location.y = 0
        if self.location.x < 0: 
            self.location.x = width
        if self.location.y < 0:
            self.location.y = height
        self.acceleration.mult(0)
        #self.display()
        
    def applyForce(self,force):
        self.acceleration.add(force)
        
    def seek(self,target):
        self.desired = PVector.sub(target,self.location)
        self.desired.normalize()
        self.desired.mult(self.maxspeed)
        steer = PVector.sub(self.desired,self.velocity)
        steer.limit(self.maxforce)
        #self.applyForce(steer)
        return steer
        
    def follow(self,flow):
        desired = flow.lookup(self.location)
        desired.mult(self.maxspeed)
        steer = PVector.sub(desired,self.velocity)
        steer.limit(self.maxforce)
        self.applyForce(steer)
        
    def separate(self,boidsList):
        desiredSeparation = self.r *2
        _sum = PVector(0,0)
        count = 0
        for boid in boidsList:
            d = PVector.dist(self.location,boid.location)
            if d > 0 and d < desiredSeparation:
                diff = PVector.sub(self.location,boid.location)
                diff.normalize()
                diff.div(d)
                _sum.add(diff)
                count += 1
        if count > 0:
            _sum.div(count)
        if _sum.mag() > 0:
            _sum.normalize()
            _sum.mult(self.maxspeed)
            _sum.sub(self.velocity)
            _sum.limit(self.maxforce)
        return _sum
            
    def align(self,boids):
        neighbordist = 50
        sum = PVector(0,0)
        count = 0
        for other in boids:
            d = PVector.dist(self.location,other.location)
            if d > 0 and d < neighbordist:
                sum.add(other.velocity)
                count += 1
        if count > 0:
            sum.div(count)
            sum.normalize()
            sum.mult(self.maxspeed)
            steer = PVector.sub(sum,self.velocity)
            steer.limit(self.maxforce)
            return steer
        else:
            return PVector(0,0)
        
    def cohesion(self,boids):
        neighbordist = 50
        sum = PVector(0,0)
        count = 0
        for other in boids:
            d = PVector.dist(self.location,other.location)
            if d > 0 and d < neighbordist:
                sum.add(other.location)
                count += 1
        if count > 0:
            sum.div(count)
            return self.seek(sum)
        else:
            return PVector(0,0)
            
    def flock(self,boidList):
        self.sep = self.separate(boidList)
        self.ali = self.align(boidList)
        self.coh = self.cohesion(boidList)
        
        #arbitrary weights. Try different ones!
        self.sep.mult(1.5)
        self.ali.mult(1.0)
        self.coh.mult(1.0)
        
        self.applyForce(self.sep)
        self.applyForce(self.ali)
        self.applyForce(self.coh)
        
    def display(self):
        theta = self.velocity.heading() + PI/2.0
        fill(175)
        stroke(0)
        pushMatrix()
        translate(self.location.x,self.location.y)
        rotate(theta)
        beginShape()
        vertex(0,-self.r*2)
        vertex(-self.r,self.r*2)
        vertex(self.r,self.r*2)
        endShape(CLOSE)
        popMatrix()
        
        
def setup():
    global myflock
    size(600,600)
    #myboid = Boid(400,400)
    myflock = Flock()
    for i in range(2):
        myflock.addBoid(Boid(width/2,height/2))
    
def draw():
    background(255) #white
    myflock.run()
    
def mouseDragged():
    myflock.addBoid(Boid(mouseX,mouseY))