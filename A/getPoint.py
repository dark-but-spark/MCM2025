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
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']  # 黑体
plt.rcParams['axes.unicode_minus'] = False 
df=pd.read_excel('E:\\2025MCM\\MCM2025\\A\\附件\\result2.xlsx',sheet_name='Sheet1')
#problem1
# state=[-math.pi,120,1.5,3.6]
#problem2
# state=[0.07746690843884262, 85.40106580280234, 1.4511953249538294, 0.0045532617264222724]
#problem3
# state=[0.0876,130.24,0,0,1,0,2,0]
#problem4
state=[0.096, 91.816, 0.478,0.633, 4.810,103.479, 7.228, 5.991,1.326, 123.794, 23.904, 1.766]
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


print(df)
colors = plt.cm.rainbow
fig,ax=plt.subplots()
for i in range(0, len(state), 4):
    direction=state[i]
    FY1_v=state[i+1]
    for j in range(1):
        FY1_tFly=state[i+j*2+2]
        FY1_tDrop=state[i+j*2+3]
        fly_init=Fly(id=i//4+1,direction=direction,v=FY1_v)
        fly=fly_init.fly(FY1_tFly)
        smokeBall=Fly(id=i//4+1,direction=direction,v=FY1_v).fly(FY1_tFly)
        t=0;dt=0.01
        # plt.plot([fly_init.x,smokeBall.x],[fly_init.z,smokeBall.z],color=colors(i/len(state)),linestyle='-',label=f'FY{i//4+1}飞行阶段')
        x=[smokeBall.x];y=[smokeBall.z]
        while t<FY1_tDrop:
            t+=dt
            smokeBall.drop_tick(dt)
            x.append(smokeBall.x);y.append(smokeBall.z)
        # print(x,y)
        smokeBall=SmokeBall(x=smokeBall.x,y=smokeBall.y,z=smokeBall.z,r=10,v=3)
        # plt.plot(x,y,color=colors((i+j*3+1)/len(state)),linestyle='--',label=f'FY{i//4+1}第{j+1}颗弹投放阶段')
        x_final=smokeBall.time(20).x;y_final=smokeBall.time(20).z
        # rect=plt.Rectangle((x_final-smokeBall.r,y_final), 2*smokeBall.r, 20*3, color=colors((i+j*3+2)/len(state)), alpha=0.5,label=f'FY{i//4+1}第{j+1}颗弹干扰范围')
        # ax.add_patch(rect)
        time=work(direction,FY1_v,FY1_tFly,FY1_tDrop,1,i//4+1)
        print(time)
    rect=plt.Rectangle((time[0],0), (time[1]-time[0]), 1, color=colors(i/len(state)), alpha=0.5,label=f'FY{i//4+1}')
    ax.add_patch(rect)
    
    df.loc[i//4,"无人机运动方向"]=direction/(2*np.pi)*360
    df.loc[i//4,"无人机运动速度 (m/s)"]=FY1_v
    df.loc[i//4,"烟幕干扰弹投放点的x坐标 (m)"]=fly.x
    df.loc[i//4,"烟幕干扰弹投放点的y坐标 (m)"]=fly.y
    df.loc[i//4,"烟幕干扰弹投放点的z坐标 (m)"]=fly.z
    df.loc[i//4,"烟幕干扰弹起爆点的x坐标 (m)"]=smokeBall.x
    df.loc[i//4,"烟幕干扰弹起爆点的y坐标 (m)"]=smokeBall.y
    df.loc[i//4,"烟幕干扰弹起爆点的z坐标 (m)"]=smokeBall.z
    df.loc[i//4,"有效干扰时长 (s)"]=time[1]-time[0]
for i in range(1):
    missile=Missile(id=i+1)
    x=missile.x/missile.vx
    print(x)
    # plt.plot([0,missile.x],[0,missile.z],color="r",linestyle='--',label=f'Missile{i+1}打击路线')
    plt.plot([x,x],[0,1],color="r",linestyle='--',label=f'Missile{i+1}打击时间')
plt.xlabel('时间 (s)')
plt.ylabel('干扰效果')
plt.title('烟幕干扰效果时间段')
# plt.xlabel('X (m)')
# plt.ylabel('Z (m)')
# plt.title('无人机飞行轨迹及烟幕干扰范围示意图')
# plt.xlim(5000,18500)
# plt.ylim(500,1850)
plt.legend()
plt.show()
# print(df)
# df.to_excel('E:\\2025MCM\\MCM2025\\A\\附件\\result2.xlsx',index=False)
