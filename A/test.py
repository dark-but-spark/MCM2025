from util.Log import Message 
from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile
import src.Checker as Checker
real= Cylinder(r=7,height=10,x=0,y=50,z=0)
M1=Missile(x=100,y=-100,z=0,v=1)
error=1e-5
t=0
smokeBall=SmokeBall(x=10,y=0,z=0)

Message("start", "INFO",client=False)
while True:
    dt=0.1
    M1.time_tick(dt)
    t+=dt
    # Message(f"t={t:.3f}s M1=({M1.x:.1f},{M1.y:.1f},{M1.z:.1f}),SmokeBall=({smokeBall.x:.1f},{smokeBall.y:.1f},{smokeBall.z:.1f})","DEBUG",client=False)
    # if not Checker.visible(checkCylinder=real,smokeBall=smokeBall,missile=M1):
        # Message(f"导弹在t={t:.3f}s看不到真目标","INFO",client=False)
    # else:
        # Message(f"导弹在t={t:.3f}s发现真目标","WARNING",client=False)
    if M1.x<-10-error:
        Message(f"导弹击中假目标！","CRITICAL",client=False)
        break