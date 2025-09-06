from util.Log import Message
from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile
from modules.Fly import Fly
import src.Checker as Checker
import math, random
from tqdm import tqdm


def work(direction,FY1_v,FY1_tFly,FY1_tDrop,dt=0.1):
    FY1=Fly(id=1,direction=direction,v=FY1_v).fly(FY1_tFly).drop(FY1_tDrop)
    M1=Missile(id=1).time(FY1_tFly+FY1_tDrop)
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
            return (-1,-1)
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
    Message(f"导弹在t={L:.3f}s到t={R:.3f}s看不到真目标,方向为{direction:.3f}rad，速度为{FY1_v:.3f}m/s，飞行时间为{FY1_tFly:.3f}s，投放时间为{FY1_tDrop:.3f}s","INFO")
    return (L,R)

def union_length(intervals):#区间并集
    ivs = [iv for iv in intervals if iv[0] >= 0.0 and iv[1] > iv[0] + error]
    if not ivs:#无有效区间
        return 0
    ivs.sort(key=lambda x: x[0])
    tot = 0.0
    curL, curR = ivs[0]
    for a, b in ivs[1:]:
        if a > curR + error:  
            tot += (curR - curL)
            curL, curR = a, b
        else:               
            if b > curR:
                curR = b
    tot += (curR - curL)
    return tot

def work_three(direction, FY1_v,
               tFly1, tDrop1,
               tFly2, tDrop2,
               tFly3, tDrop3,
               dt_coarse=0.3,
               overlap_penalty=0.2):
    """
    返回 union_len - λ*overlap
    union_len 为有效区间的并集长度 overlap = sum_len - union_len
    只要任一枚有效即可参与并集；全无效则返回 0.0
    """
    tFly1, tFly2, tFly3 = project_spacing(tFly1, tFly2, tFly3, delta=1.0)
    iv1 = work(direction, FY1_v, tFly1, tDrop1, dt=dt_coarse)
    iv2 = work(direction, FY1_v, tFly2, tDrop2, dt=dt_coarse)
    iv3 = work(direction, FY1_v, tFly3, tDrop3, dt=dt_coarse)

    valid = [iv for iv in (iv1, iv2, iv3) if iv[0] >= 0.0]
    if not valid:
        return 0.0

    union_len = union_length(valid)
    sum_len   = sum((b - a) for (a, b) in valid)
    overlap   = max(0.0, sum_len - union_len)
    score     = union_len - overlap_penalty * overlap
    return max(0.0, score)

def update_best():
    global maxTime, direction_best, FY1_v_best
    global tFly1_best, tDrop1_best, tFly2_best, tDrop2_best, tFly3_best, tDrop3_best
    maxTime = nowTime
    direction_best = direction
    FY1_v_best = FY1_v
    tFly1_best, tDrop1_best = tFly1, tDrop1
    tFly2_best, tDrop2_best = tFly2, tDrop2
    tFly3_best, tDrop3_best = tFly3, tDrop3
    Message(f"[best] score={maxTime:.3f}, dir={direction_best:.3f}, v={FY1_v_best:.2f}, "
            f"tFly=({tFly1_best:.2f},{tFly2_best:.2f},{tFly3_best:.2f}), "
            f"tDrop=({tDrop1_best:.2f},{tDrop2_best:.2f},{tDrop3_best:.2f})", "INFO")

def update(direction_new, FY1_v_new,
            tFly1_new, tDrop1_new,
            tFly2_new, tDrop2_new,
            tFly3_new, tDrop3_new):
    global direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3
    direction = direction_new
    FY1_v     = FY1_v_new
    tFly1, tDrop1 = tFly1_new, tDrop1_new
    tFly2, tDrop2 = tFly2_new, tDrop2_new
    tFly3, tDrop3 = tFly3_new, tDrop3_new

def project_spacing(t1, t2, t3, delta=1.0):
    ts = sorted([t1, t2, t3])
    if ts[1] < ts[0] + delta:
        ts[1] = ts[0] + delta
    if ts[2] < ts[1] + delta:
        ts[2] = ts[1] + delta
    ts = [max(0.0, x) for x in ts]
    # 再次校正可能被非负钳位破坏的间隔
    if ts[1] < ts[0] + delta:
        ts[1] = ts[0] + delta
    if ts[2] < ts[1] + delta:
        ts[2] = ts[1] + delta
    return ts[0], ts[1], ts[2]

def clamp_nonneg(x): return x if x >= 0.0 else 0.0
def clamp(x, lo, hi): return lo if x < lo else (hi if x > hi else x)

def choose_new(step):
    global direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3
    s0  = step * random.uniform(-1, 1)
    th1 = random.uniform(0, 2*math.pi)
    th2 = random.uniform(0, 2*math.pi)
    th3 = random.uniform(0, 2*math.pi)
    th4 = random.uniform(0, 2*math.pi)
    th5 = random.uniform(0, 2*math.pi)
    th6 = random.uniform(0, 2*math.pi)
    direction_new = direction + s0 * math.sin(th1) * math.sin(th2) * math.pi
    if direction_new < 0: direction_new += 2*math.pi
    elif direction_new >= 2*math.pi: direction_new -= 2*math.pi
    FY1_v_new = clamp(FY1_v + s0 * math.sin(th1) * math.cos(th2) * 20.0, 70.0, 140.0)

    tFly1_new = tFly1 + s0 * math.cos(th1) * math.sin(th3) * 5.0
    tFly2_new = tFly2 + s0 * math.cos(th1) * math.cos(th3) * 5.0
    tFly3_new = tFly3 + s0 * math.cos(th1) * math.sin(th4) * 5.0
    tFly1_new, tFly2_new, tFly3_new = project_spacing(tFly1_new, tFly2_new, tFly3_new, delta=1.0)

    tDrop1_new = clamp_nonneg(tDrop1 + s0 * math.cos(th1) * math.cos(th4) * 2.0)
    tDrop2_new = clamp_nonneg(tDrop2 + s0 * math.sin(th5) * math.sin(th4) * 2.0)
    tDrop3_new = clamp_nonneg(tDrop3 + s0 * math.cos(th6) * math.cos(th3) * 2.0)

    return [direction_new, FY1_v_new,
            tFly1_new, tDrop1_new,
            tFly2_new, tDrop2_new,
            tFly3_new, tDrop3_new]


Message("开始运行 Problem3", "INFO")
real  = Cylinder(r=7, height=10, x=0, y=200, z=0)
error = 1e-5
INIT_MAX   = 180.0   # 初始投放时刻
DROP_INIT  = 60.0   # 初始延时范围上界（仅用于采样初值，非硬约束）
direction = random.uniform(0, 2*math.pi)
FY1_v     = random.uniform(70, 140)

tFly1 = random.uniform(0.0, INIT_MAX - 2.0)
tFly2 = tFly1 + 1.0 + random.uniform(0.0, 1.0)
tFly3 = tFly2 + 1.0 + random.uniform(0.0, 1.0)
tFly1, tFly2, tFly3 = project_spacing(tFly1, tFly2, tFly3, delta=1.0)

tDrop1 = random.uniform(0.0, DROP_INIT)
tDrop2 = random.uniform(0.0, DROP_INIT)
tDrop3 = random.uniform(0.0, DROP_INIT)

nowTime = work_three(direction, FY1_v,
                          tFly1, tDrop1,
                          tFly2, tDrop2,
                          tFly3, tDrop3,
                          dt_coarse=0.2)
# <<<<<< 在这里插入自检打印 >>>>>>
ivs0 = [
        work(direction, FY1_v, tFly1, tDrop1, dt=0.2),
        work(direction, FY1_v, tFly2, tDrop2, dt=0.2),
        work(direction, FY1_v, tFly3, tDrop3, dt=0.2),
        ]
Message(f"[init intervals] {ivs0}", "INFO")
# <<<<<< 自检结束 >>>>>>
maxTime = -1.0
direction_best = FY1_v_best = 0.0
tFly1_best = tDrop1_best = tFly2_best = tDrop2_best = tFly3_best = tDrop3_best = 0.0

if nowTime > 0:
    update_best()
else:
    Message("初始化 score=0，进入退火搜索", "INFO")

T= 2000.0
alpha= math.exp(-1e-2)
step=5

for i in tqdm(range(3000)):
    direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new = choose_new(step)
    predictTime = work_three(direction_new, FY1_v_new,
                                 tFly1_new, tDrop1_new,
                                 tFly2_new, tDrop2_new,
                                 tFly3_new, tDrop3_new,
                                 dt=0.2)
    while predictTime < 0:
        direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new = choose_new(step)
        predictTime = work_three(direction_new, FY1_v_new,
                                 tFly1_new, tDrop1_new,
                                 tFly2_new, tDrop2_new,
                                 tFly3_new, tDrop3_new,
                                 dt=0.2)
    delta = predictTime - nowTime
    if delta > 0 or math.exp(delta / T) > random.random():
        update(direction_new, FY1_v_new,
                    tFly1_new, tDrop1_new,
                    tFly2_new, tDrop2_new,
                    tFly3_new, tDrop3_new)
        nowTime = predictTime
        if nowTime> maxTime:
                update_best()

    T*= alpha
    step*= alpha
    if step <= error:
            break

Message(f"最终最优 score={maxTime:.3f}, dir={direction_best:.3f}, v={FY1_v_best:.2f}, "
            f"tFly=({tFly1_best:.2f},{tFly2_best:.2f},{tFly3_best:.2f}), "
            f"tDrop=({tDrop1_best:.2f},{tDrop2_best:.2f},{tDrop3_best:.2f})", "INFO")
Message("运行结束 Problem3", "INFO")
