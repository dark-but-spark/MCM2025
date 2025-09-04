import math
class Fly:
    def __init__(self, x, y, z, v=0, direction=0):
        '''
        x,y,z:初始位置
        v:速度
        direction:飞行方向（弧度制，0为x轴正方向，逆时针旋转）
         '''
        self.x=x
        self.y=y
        self.z=z
        self.vx=v*math.cos(direction)
        self.vy=v*math.sin(direction)
        self.vz=0
    def fly_tick(self,dt):#平飞一段时间
        self.x+=self.vx*dt
        self.y+=self.vy*dt
    def drop_tick(self,dt): #自由落体一段时间
        self.x+=self.vx*dt
        self.y+=self.vy*dt
        self.z=self.z-0.5*9.81*dt*dt+self.vz*dt
        self.vz-=9.81*dt
    def fly(self,t):#平飞未来t秒后的位置
        F_new=Fly(self.x,self.y,self.z,0,0)
        F_new.vx=self.vx
        F_new.vy=self.vy
        F_new.vz=self.vz
        F_new.x+=F_new.vx*t
        F_new.y+=F_new.vy*t
        return F_new
    def drop(self,t):#自由落体未来t秒后的位置
        F_new=Fly(self.x,self.y,self.z,0,0)
        F_new.vx=self.vx
        F_new.vy=self.vy
        F_new.vz=self.vz
        F_new.x+=F_new.vx*t
        F_new.y+=F_new.vy*t
        F_new.z=F_new.z-0.5*9.81*t*t+F_new.vz*t
        F_new.vz-=9.81*t
        return F_new