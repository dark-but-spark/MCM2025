from copy import deepcopy
from util.Log import Message 
from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile
from modules.Fly import Fly
import src.Checker as Checker
import math
import numpy as np
import random
from tqdm import tqdm
import pandas as pd
import mathplotlib.pyplot as plt
df=pd.read_excel('E:\\2025MCM\\MCM2025\\A\\附件\\result2.xlsx',sheet_name='Sheet1')
state=[0.09090288302295513, 137.20475950387242, 0.7426129342663543, 0.15667486701543254, 5.284121957171391, 109.89442845023504, 10.267618574371745, 4.601809922977203, 1.3188654458199724, 130.12437350784376, 23.69236887045316, 0.7197184278257467]
error=1e-5
real= Cylinder(r=7,height=10,x=0,y=200,z=0)
def work(direction,FY1_v,FY1_tFly,FY1_tDrop,Mid,FYid,dt=0.1): #计算某个烟雾遮某个导弹的时间
    FY1=Fly(id=FYid,direction=direction,v=FY1_v).fly(FY1_tFly).drop(FY1_tDrop)
    M1=Missile(id=Mid).time(FY1_tFly+FY1_tDrop)
    smokeBall=SmokeBall(x=FY1.x,y=FY1.y,z=FY1.z,r=10,v=3)
    t=FY1_tFly+FY1_tDrop
    LL=FY1_tFly+FY1_tDrop;LR=FY1_tFly+FY1_tDrop
    RL=FY1_tFly+FY1_tDrop;RR=20+FY1_tFly+FY1_tDrop
    # if not Checker.visible(checkCylinder=real,smokeBall=smokeBall,missile=M1):
    
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
    # while True:
    #     t+=dt
    #     M1.time_tick(dt)
    #     smokeBall.time_tick(dt)
    #     if not Checker.visible(checkCylinder=real,smokeBall=smokeBall,missile=M1):
    #         Message(f"当t={t:.3f}s时，烟幕能保护目标","DEBUG")
    #     if t>FY1_tFly+FY1_tDrop+20+error:
    #         break
    L=LR
    while LR-LL>error:
        t=(LL+LR)/2
        M_new=Missile(id=1).time(t)
        smokeBall_new=SmokeBall(x=FY1.x,y=FY1.y,z=FY1.z,r=10,v=3).time(t-FY1_tFly-FY1_tDrop)
        if not Checker.visible(checkCylinder=real,smokeBall=smokeBall_new,missile=M_new):
            L=LR=t
            # Message(f"当t={t:.3f}s时，烟幕能保护目标","DEBUG")
        else:
            LL=t
    R=RL
    while RR-RL>error:
        t=(RL+RR)/2
        M_new=Missile(id=1).time(t)
        smokeBall_new=SmokeBall(x=FY1.x,y=FY1.y,z=FY1.z,r=10,v=3).time(t-FY1_tFly-FY1_tDrop)
        if not Checker.visible(checkCylinder=real,smokeBall=smokeBall_new,missile=M_new):
            R=RL=t
            # Message(f"当t={t:.3f}s时，烟幕能保护目标","DEBUG")
        else:
            RR=t
    Message(f"导弹在t={L:.3f}s到t={R:.3f}s看不到真目标,方向为{direction:.3f}rad，FY{FYid}速度为{FY1_v:.3f}m/s，飞行时间为{FY1_tFly:.3f}s，投放时间为{FY1_tDrop:.3f}s","INFO")
    return [L,R]

df=pd.read_excel('E:\\2025MCM\\MCM2025\\A\\附件\\result2.xlsx',sheet_name='Sheet1')
state=[0.09090288302295513, 137.20475950387242, 0.7426129342663543, 0.15667486701543254, 5.284121957171391, 109.89442845023504, 10.267618574371745, 4.601809922977203, 1.3188654458199724, 130.12437350784376, 23.69236887045316, 0.7197184278257467]
print(df)

for i in range(0, len(state), 4):
    direction=state[i]
    FY1_v=state[i+1]
    FY1_tFly=state[i+2]
    FY1_tDrop=state[i+3]
    fly=Fly(id=i//4+1,direction=direction,v=FY1_v).fly(FY1_tFly)
    smokeBall=Fly(id=i//4+1,direction=direction,v=FY1_v).fly(FY1_tFly).drop(FY1_tDrop)
    time=work(direction,FY1_v,FY1_tFly,FY1_tDrop,1,i//4+1)
    print(time)
    df.loc[i//4,"无人机运动方向"]=direction/(2*np.pi)*360
    df.loc[i//4,"无人机运动速度 (m/s)"]=FY1_v
    df.loc[i//4,"烟幕干扰弹投放点的x坐标 (m)"]=fly.x
    df.loc[i//4,"烟幕干扰弹投放点的y坐标 (m)"]=fly.y
    df.loc[i//4,"烟幕干扰弹投放点的z坐标 (m)"]=fly.z
    df.loc[i//4,"烟幕干扰弹起爆点的x坐标 (m)"]=smokeBall.x
    df.loc[i//4,"烟幕干扰弹起爆点的y坐标 (m)"]=smokeBall.y
    df.loc[i//4,"烟幕干扰弹起爆点的z坐标 (m)"]=smokeBall.z
    df.loc[i//4,"有效干扰时长 (s)"]=time[1]-time[0]
print(df)
df.to_excel('E:\\2025MCM\\MCM2025\\A\\附件\\result2.xlsx',index=False)
