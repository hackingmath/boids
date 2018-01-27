class Flock:
    def __init__(self):
        self.boids = []
        
    def run(self):
        for b in self.boids:
            b.run(self.boids)
            
    def addBoid(self,b):
        self.boids.append(b)