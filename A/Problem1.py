from util.Log import Message 
from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile
from modules.Fly import Fly
import src.Checker as Checker
import math
import random


def work(dirction):
    FY1=Fly(x=17800,y=0,z=1800,direction=dirction,v=FY1_v).fly(FY1_tFly).drop(FY1_tDrop)
    M1.time_tick(FY1_tFly+FY1_tDrop)
    t=FY1_tFly+FY1_tDrop
    

Message("开始运行Problem1","INFO")
real= Cylinder(r=7,height=10,x=0,y=200,z=0)
FY1_v=120
FY1_tFly=1.5
FY1_tDrop=3.6
M1=Missile(x=20000,y=0,z=2000)
error=1e-5
t=0
def e(n):
    return math.exp(-n/1000)

direction=random.uniform(0,math.pi)
step=1
for i in range(10000):
    