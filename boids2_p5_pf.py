'''Autonomous Agents from Nature of Code
January 29, 2017
p5.py version

Error message:
Warning (from warnings module):
  File "C:\\Users\\pfarrell\\AppData\\Roaming\\Python\\Python36\\site-packages\\p5\\pmath\\vector.py", line 321
    return np.arccos( (self @ other) / (self.magnitude * other.magnitude))
RuntimeWarning: invalid value encountered in float_scalars
'''

from flock import Flock
from p5 import *

class Boid:
    def __init__(self,x,y):
        self.location = Vector(x,y)
        self.velocity = Vector(0,0)
        self.acceleration = Vector(0,0)
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
        self.velocity += self.acceleration
        self.velocity.limit(self.maxspeed)
        self.location += self.velocity
        if self.location.x > width:
            self.location.x = 0
        if self.location.y > height:
            self.location.y = 0
        if self.location.x < 0: 
            self.location.x = width
        if self.location.y < 0:
            self.location.y = height
        self.acceleration *= 0
        #self.display()
        
    def applyForce(self,force):
        self.acceleration += force
        
    def seek(self,target):
        self.desired = target - self.location
        self.desired.normalize()
        self.desired *= self.maxspeed
        steer = self.desired - self.velocity
        steer.limit(self.maxforce)
        #self.applyForce(steer)
        return steer
        
    def follow(self,flow):
        desired = flow.lookup(self.location)
        desired *= self.maxspeed
        steer = desired - self.velocity
        steer.limit(self.maxforce)
        self.applyForce(steer)
        
    def separate(self,boidsList):
        desiredSeparation = self.r *2
        _sum = Vector(0,0)
        count = 0
        for boid in boidsList:
            d = dist((self.location[0],self.location[1]),
                     (boid.location[0],boid.location[1]))
            if d > 0 and d < desiredSeparation:
                diff = self.location - boid.location
                diff.normalize()
                diff /= d
                _sum += diff
                count += 1
        if count > 0:
            _sum /= count
        if _sum.magnitude > 0:
            _sum.normalize()
            _sum *= self.maxspeed
            _sum -= self.velocity
            _sum.limit(self.maxforce)
        return _sum
            
    def align(self,boids):
        neighbordist = 50
        _sum = Vector(0,0)
        count = 0
        for other in boids:
            d = dist((self.location[0],self.location[1]),
                     (other.location[0],other.location[1]))
            if d > 0 and d < neighbordist:
                _sum += other.velocity
                count += 1
        if count > 0:
            _sum /= count
            _sum.normalize()
            _sum.mult(self.maxspeed)
            steer = _sum - self.velocity
            steer.limit(self.maxforce)
            return steer
        else:
            return Vector(0,0)
        
    def cohesion(self,boids):
        neighbordist = 50
        _sum = Vector(0,0)
        count = 0
        for other in boids:
            d = dist((self.location[0],self.location[1]),
                     (other.location[0],other.location[1]))
            if d > 0 and d < neighbordist:
                _sum += other.location
                count += 1
        if count > 0:
            _sum /= count
            return self.seek(_sum)
        else:
            return Vector(0,0)
            
    def flock(self,boidList):
        self.sep = self.separate(boidList)
        self.ali = self.align(boidList)
        self.coh = self.cohesion(boidList)
        
        #arbitrary weights. Try different ones!
        self.sep *= 1.5
        self.ali *= 1.0
        self.coh *= 1.0
        
        self.applyForce(self.sep)
        self.applyForce(self.ali)
        self.applyForce(self.coh)
        
    def display(self):
        theta = self.velocity.angle_between(Vector(1,0))
        fill(175)
        stroke(0)
        push_matrix()
        translate(self.location.x,self.location.y)
        rotate(theta)
        triangle((0,-self.r*2),
                (-self.r,self.r*2),
                 (self.r,self.r*2))
        reset_matrix()
        
        
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

run()
