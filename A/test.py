from util.Log import Message 
from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile
from modules.Fly import Fly
import src.Checker as Checker
import math
import random
real= Cylinder(r=7,height=10,x=0,y=50,z=0)
M1=Missile(x=100,y=-100,z=0,v=1)
error=1e-5
t=0
smokeBall=SmokeBall(x=50,y=-50,z=0)

Message("start", "INFO")
while True:
    dt=0.1
    M1.time_tick(dt)
    t+=dt
    Message(f"t={t:.3f}s M1=({M1.x:.1f},{M1.y:.1f},{M1.z:.1f}),SmokeBall=({smokeBall.x:.1f},{smokeBall.y:.1f},{smokeBall.z:.1f})","DEBUG")
    if not Checker.visible(checkCylinder=real,smokeBall=smokeBall,missile=M1):
        Message(f"导弹在t={t:.3f}s看不到真目标","INFO")
    else:
        Message(f"导弹在t={t:.3f}s发现真目标","WARNING",)
    if M1.x<-5-error:
        Message(f"导弹击中假目标！","CRITICAL",client=False)
        break

# t1=random.uniform(0,10)
# t2=random.uniform(0,10)
t1=10
t2=10
F1=Fly(x=0,y=0,z=0,v=10,direction=math.pi/4)
F1_fly=F1.fly(t1)
F1_flyAndDrop=F1.fly(t1).drop(t2)
Message(f"飞行器平飞{t1:.3f}s后的位置为({F1_fly.x:.3f},{F1_fly.y:.3f},{F1_fly.z:.3f})","INFO")
Message(f"飞行器先平飞{t1:.3f}s再丢下烟雾弹{t2:.3f}s后的位置为({F1_flyAndDrop.x:.3f},{F1_flyAndDrop.y:.3f},{F1_flyAndDrop.z:.3f})","INFO")