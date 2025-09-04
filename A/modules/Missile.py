class Missile:
    # 导弹来袭 从x,y,z位置以300m/s的速度向原点飞去
    def __init__(self, x, y, z, v=300):
        self.x = x
        self.y = y
        self.z = z
        self.vx = v*(self.x)/((self.x)**2+(self.y)**2+(self.z)**2)**0.5
        self.vy = v*(self.y)/((self.x)**2+(self.y)**2+(self.z)**2)**0.5
        self.vz = v*(self.z)/((self.x)**2+(self.y)**2+(self.z)**2)**0.5
    def time_tick(self, dt): #时间滴答滴答 你的论文写完了吗 存在误差
        self.x -= dt* self.vx
        self.y -= dt* self.vy
        self.z -= dt* self.vz
    def time(self,t):#未来t秒后的位置 还是有误差
        M_new=Missile(self.x,self.y,self.z)
        M_new.x-=t*M_new.vx
        M_new.y-=t*M_new.vy
        M_new.z-=t*M_new.vz
        return M_new
#误差分析 由于一个坐标过大导致计算上的误差