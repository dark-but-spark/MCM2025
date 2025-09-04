class Cylinder:
    def __init__(self, r, height,x,y,z):
        self.r = r
        self.height = height
        self.x=x
        self.y=y
        self.z=z
    def random_point_on_surface(self):
        import random
        import math
        face=random.uniform(0,1)
        if(face<(math.pi*self.r*self.r)/(math.pi*self.r*self.height+math.pi*self.r*self.r)):
            #top or bottom face
            theta = random.uniform(0, 2 * math.pi)
            r0 = self.r * math.sqrt(random.uniform(0, 1))
            x = self.x + r0 * math.cos(theta)
            y = self.y + r0 * math.sin(theta)
            z = self.z + random.choice([0, self.height])
        else:
            #curved surface
            theta = random.uniform(0, 2 * math.pi)
            z = self.z + random.uniform(0, self.height)
            x = self.x + self.r * math.cos(theta)
            y = self.y + self.r * math.sin(theta)
        return (x, y, z)