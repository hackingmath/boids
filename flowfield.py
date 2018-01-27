class Flowfield:
    def __init__(self,r):
        self.resolution = r
        self.cols = int(width/self.resolution)
        self.rows = int(height/self.resolution)
        self.field = []
        self.initialize() #Shiffman named it "init"!
        
        
    def initialize(self):
        self.xoff = 0
        for i in range(self.cols):
            self.field.append([]) #empty list for row
            self.yoff = 0
            for j in range(self.rows):
                theta = map(noise(self.xoff,self.yoff),0,1,0,TWO_PI)
                self.field[i].append(PVector(cos(theta),
                                          sin(theta)))
                self.yoff += 0.1
            self.xoff += 0.1
    
    def lookup(self,lookup):
        column = int(constrain(lookup.x/self.resolution,0,self.cols-1))
        row = int(constrain(lookup.y/self.resolution,0,self.rows-1))
        return self.field[column][row]#.get()