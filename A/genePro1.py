from util.Log import Message 
from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile
from modules.Fly import Fly
import src.Checker as Checker
import math
import random
from tqdm import tqdm


def work(direction,FY1_v,FY1_tFly,FY1_tDrop,dt=0.1,FYid=1,Mid=1): #计算某个烟雾遮某个导弹的时间
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
            return 0
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
        M_new=Missile(id=Mid).time(t)
        smokeBall_new=SmokeBall(x=FY1.x,y=FY1.y,z=FY1.z,r=10,v=3).time(t-FY1_tFly-FY1_tDrop)
        if not Checker.visible(checkCylinder=real,smokeBall=smokeBall_new,missile=M_new):
            L=LR=t
            # Message(f"当t={t:.3f}s时，烟幕能保护目标","DEBUG")
        else:
            LL=t
    R=RL
    while RR-RL>error:
        t=(RL+RR)/2
        M_new=Missile(id=Mid).time(t)
        smokeBall_new=SmokeBall(x=FY1.x,y=FY1.y,z=FY1.z,r=10,v=3).time(t-FY1_tFly-FY1_tDrop)
        if not Checker.visible(checkCylinder=real,smokeBall=smokeBall_new,missile=M_new):
            R=RL=t
            # Message(f"当t={t:.3f}s时，烟幕能保护目标","DEBUG")
        else:
            RR=t
    Message(f"导弹在t={L:.3f}s到t={R:.3f}s看不到真目标,方向为{direction:.3f}rad，速度为{FY1_v:.3f}m/s，飞行时间为{FY1_tFly:.3f}s，投放时间为{FY1_tDrop:.3f}s","INFO")
    return R-L
def update_best():
    global maxTime,direction_best,FY1_v_best,FY1_tFly_best,FY1_tDrop_best
    maxTime=nowTime
    direction_best=direction
    FY1_v_best=FY1_v
    FY1_tFly_best=FY1_tFly
    FY1_tDrop_best=FY1_tDrop
    Message(f"目前最优解，烟幕保护时间为{maxTime:.3f}s，方向为{direction_best:.3f}rad，速度为{FY1_v_best:.3f}m/s，飞行时间为{FY1_tFly_best:.3f}s，投放时间为{FY1_tDrop_best:.3f}s","INFO")
def update(direction_new,FY1_v_new,FY1_tFly_new,FY1_tDrop_new):
    global direction,FY1_v,FY1_tFly,FY1_tDrop
    direction=direction_new
    FY1_v=FY1_v_new
    FY1_tFly=FY1_tFly_new
    FY1_tDrop=FY1_tDrop_new
def choose_new(step):
    global direction,FY1_v,FY1_tFly,FY1_tDrop
    step0=step*random.uniform(-1,1)
    theta1=random.uniform(0,2*math.pi)
    theta2=random.uniform(0,2*math.pi)
    theta3=random.uniform(0,2*math.pi)
    direction_new=direction+step0*math.sin(theta1)*math.sin(theta2)*math.pi
    if direction_new<0:
        direction_new+=2*math.pi
    elif direction_new>=2*math.pi:
        direction_new-=2*math.pi
    FY1_v_new=(FY1_v+step0*math.sin(theta1)*math.cos(theta2)*35)%70+70
    FY1_tFly_new=(FY1_tFly+step0*math.cos(theta1)*math.sin(theta3)*10+35)%35
    FY1_tDrop_new=(FY1_tDrop+step0*math.cos(theta1)*math.cos(theta3)*10+35)%35
    return [direction_new,FY1_v_new,FY1_tFly_new,FY1_tDrop_new ]
def get_new_gene():
    for _ in range(10000000):
        direction=random.uniform(0,2*math.pi)
        FY1_v=random.uniform(70,140)
        t1=random.uniform(0,70)
        t11=max(random.uniform(0,70-t1),0)
        t2=t1+max(random.uniform(0,69-t1),0)+1
        t21=max(random.uniform(0,70-t2),0)
        t3=t2+max(random.uniform(0,69-t2),0)+1
        t31=max(random.uniform(0,70-t3),0)
        t=[[t1,t11],[t2,t21],[t3,t31]]
        flag=0
        point_flag=[]
        for i in range(3):
            FY1_tFly=t[i][0]
            FY1_tDrop=t[i][1]
            nowTime=0
            for j in range(3):
                nowTime+=work(direction,FY1_v,FY1_tFly,FY1_tDrop,FYid=FYid,Mid=j+1)
                if(nowTime>0):
                    break
            else:
                flag+=1
                point_flag.append([])
                continue
            FY=Fly(id=FYid,direction=direction,v=FY1_v).fly(FY1_tFly).drop(FY1_tDrop)
            for point in ans:
                if (FY.x-point[0])**2+(FY.y-point[1])**2+(FY.z-point[2])**2<100:
                    flag+=1
                    point_flag.append([])
                    continue
            point_flag.append([FY.x,FY.y,FY.z])
        if( flag==3):
            continue
        for i in range(3):
            if(point_flag[i]!=[]):
                ans.append(point_flag[i])
        return [direction,FY1_v,t[0][0],t[0][1],t[1][0],t[1][1],t[2][0],t[2][1],nowTime]
            
    return -1
        
        
import os

folder = "data"
if not os.path.exists(folder):
    os.makedirs(folder)

Message("开始运行搜索优质基因","INFO")
real= Cylinder(r=7,height=10,x=0,y=200,z=0)
error=1e-5
ans=[]*3
FYid=1
Message(f"开始计算FY{FYid}","INFO")
ans=[]*3
for _ in tqdm(range(100),desc=f"FY{FYid}"):
    result=get_new_gene()
    if result==-1:
        break
    open(f"data/new_FY{FYid}_genePro.txt","a").write(f"{result[0]} {result[1]} {result[2]} {result[3]} {result[4]} {result[5]} {result[6]} {result[7]} {result[8]}\n")
    Message(f"找到一个基因，烟幕保护时间为{result[8]:.3f}s，FY{FYid}方向为{result[0]:.3f}rad，速度为{result[1]:.3f}m/s，飞行时间1为{result[2]:.3f}s，投放时间1为{result[3]:.3f}s, 飞行时间2为{result[4]:.3f}s，投放时间2为{result[5]:.3f}s，飞行时间3为{result[6]:.3f}s，投放时间3为{result[7]:.3f}s","INFO")


Message("运行结束搜索优质基因","INFO")

