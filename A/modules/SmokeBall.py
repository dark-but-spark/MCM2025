class SmokeBall:
    def __init__(self, x=0, y=0, z=0, r=10, v=3):
        self.r = r
        self.x = x
        self.y = y
        self.z = z
        self.v = v
    def time_tick(self, dt):#时间滴答滴答 你的论文写完了吗
        self.z-=dt*self.v
    def time(self,t):#未来t秒后的位置
        S_new=SmokeBall(self.x,self.y,self.z,self.r,self.v)
        S_new.z-=t*S_new.v
        return S_new
    