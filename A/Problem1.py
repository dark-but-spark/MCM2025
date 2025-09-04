from util.Log import Message 
from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile
from modules.Fly import Fly
import src.Checker as Checker
import math
import random


def work(dirction,dt=0.1):
    FY1=Fly(x=17800,y=0,z=1800,direction=dirction,v=FY1_v).fly(FY1_tFly).drop(FY1_tDrop)
    M1.time_tick(FY1_tFly+FY1_tDrop)
    smokeBall=SmokeBall(x=FY1.x,y=FY1.y,z=FY1.z,r=10,v=3)
    t=FY1_tFly+FY1_tDrop
    LL=0;LR=0
    RL=0;RR=20+FY1_tFly+FY1_tDrop
    while True:
        t+=dt
        M1.time_tick(dt)
        smokeBall.time_tick(dt)
        if not Checker.visible(checkCylinder=real,smokeBall=smokeBall,missile=M1):
            LL=t-dt;LR=t
            RL=t
            break
        if t>FY1_tFly+FY1_tDrop+20+error:
            # Message(f"未找到答案，烟幕20s内没有保护目标,方向为{direction:.3f}rad","INFO")
            return -1
    while True:
        t+=dt
        M1.time_tick(dt)
        smokeBall.time_tick(dt)
        if not Checker.visible(checkCylinder=real,smokeBall=smokeBall,missile=M1):
            Message(f"当t={t:.3f}s时，烟幕能保护目标","INFO")
        if t>FY1_tFly+FY1_tDrop+20+error:
            # Message(f"未找到答案，烟幕20s内没有保护目标,方向为{direction:.3f}rad","INFO")
            break
    L=LR
    while LR-LL>error:
        t=(LL+LR)/2
        M_new=Missile(x=20000,y=0,z=2000)
        M_new.time_tick(t)
        smokeBall_new=SmokeBall(x=FY1.x,y=FY1.y,z=FY1.z,r=10,v=3)
        smokeBall_new.time_tick(t-FY1_tFly-FY1_tDrop)
        if not Checker.visible(checkCylinder=real,smokeBall=smokeBall_new,missile=M_new):
            L=LR=t
        else:
            LL=t
    R=RL
    while RR-RL>error:
        t=(RL+RR)/2
        M_new=Missile(x=20000,y=0,z=2000)
        M_new.time_tick(t)
        smokeBall_new=SmokeBall(x=FY1.x,y=FY1.y,z=FY1.z,r=10,v=3)
        smokeBall_new.time_tick(t-FY1_tFly-FY1_tDrop)
        if not Checker.visible(checkCylinder=real,smokeBall=smokeBall_new,missile=M_new):
            R=RL=t
        else:
            RR=t
    Message(f"导弹在t={L:.3f}s到t={R:.3f}s看不到真目标,方向为{direction:.3f}rad","INFO")
    return R-L
            
    

Message("开始运行Problem1","INFO")
real= Cylinder(r=7,height=10,x=0,y=200,z=0)
FY1_v=120
FY1_tFly=1.5
FY1_tDrop=3.6
M1=Missile(x=20000,y=0,z=2000)
error=1e-5

T=1000
alpha=math.exp(1/10)
direction=random.uniform(0,math.pi)
step=1
nowTime=work(direction)
while nowTime<0:
    direction=random.uniform(0,math.pi)
    nowTime=work(direction)
maxTime=nowTime
direction_best=direction
for i in range(10000):
    direction_new=direction+random.uniform(-step,step)
    if direction_new<0:
        direction_new+=2*math.pi
    elif direction_new>=2*math.pi:
        direction_new-=2*math.pi
    predictTime=work(direction_new)
    delta=predictTime-nowTime
    if delta>0 or math.exp(delta/T)>random.uniform(0,1):
        direction=direction_new
        nowTime=predictTime
        if nowTime>maxTime:
            maxTime=nowTime
            direction_best=direction
            Message(f"找到更优解，烟幕保护时间为{maxTime:.3f}s，方向为{direction_best:.3f}rad","INFO")
    T=T*alpha
Message(f"最终结果，烟幕保护时间为{maxTime:.3f}s，方向为{direction_best:.3f}rad","INFO")
Message("运行结束Problem1","INFO")
        
            
