from util.Log import Message
from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile
from modules.Fly import Fly
import src.Checker as Checker
import math, random
from tqdm import tqdm


def work_interval(direction, FY1_v, FY1_tFly, FY1_tDrop, dt=0.1):
    FY1 = Fly(id=1, direction=direction, v=FY1_v).fly(FY1_tFly).drop(FY1_tDrop)
    M1 = Missile(id=1).time(FY1_tFly + FY1_tDrop) # 导弹推进
    smokeBall = SmokeBall(x=FY1.x, y=FY1.y, z=FY1.z, r=10, v=3)

    t = FY1_tFly + FY1_tDrop
    LL = LR = RL = FY1_tFly + FY1_tDrop
    RR = 20 + FY1_tFly + FY1_tDrop

    while True:
        t += dt
        M1.time_tick(dt)
        smokeBall.time_tick(dt)
        if not Checker.visible(checkCylinder=real, smokeBall=smokeBall, missile=M1):
            LL = t - dt; LR = t; RL = t
            break
        if t > FY1_tFly + FY1_tDrop + 20 + error:
            #Message(f"未找到答案，烟幕20s内没有保护目标,方向为{direction:.3f}rad","INFO")
            return (-1, -1)

    L = LR
    while LR - LL > error:
        t = (LL + LR) / 2
        M_new = Missile(id=1).time(t)
        smokeBall_new = SmokeBall(x=FY1.x, y=FY1.y, z=FY1.z, r=10, v=3).time(t - FY1_tFly - FY1_tDrop)
        if not Checker.visible(checkCylinder=real, smokeBall=smokeBall_new, missile=M_new):
            L = LR = t
        else:
            LL = t

    R = RL
    while RR - RL > error:
        t = (RL + RR) / 2
        M_new = Missile(id=1).time(t)
        smokeBall_new = SmokeBall(x=FY1.x, y=FY1.y, z=FY1.z, r=10, v=3).time(t - FY1_tFly - FY1_tDrop)
        if not Checker.visible(checkCylinder=real, smokeBall=smokeBall_new, missile=M_new):
            R = RL = t
        else:
            RR = t

    return (L, R)


def union_length(intervals):# 找交集
    ivs = [iv for iv in intervals if iv[0] >= 0 and iv[1] >= 0 and iv[1] > iv[0] + error]  # 过滤非法区间
    if not ivs:
        return -1
    ivs.sort(key=lambda x: x[0]) #按起点排序
    tot = 0
    curL, curR = ivs[0]
    for a, b in ivs[1:]:
        if a > curR + error:#没有重叠
            tot += (curR - curL)
            curL, curR = a, b
        else:
            if b > curR:
                curR = b
    tot += (curR - curL)
    return tot

def work_three(direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3, dt=0.1):
    (tFly1, tFly2, tFly3) = project_spacing(tFly1, tFly2, tFly3, delta=1.0)

    iv1 = work_interval(direction, FY1_v, tFly1, tDrop1, dt=dt)
    iv2 = work_interval(direction, FY1_v, tFly2, tDrop2, dt=dt)
    iv3 = work_interval(direction, FY1_v, tFly3, tDrop3, dt=dt)
    if iv1==(-1,-1) and iv2==(-1,-1) and iv3==(-1,-1):
        return -1
    ulen = union_length([iv1, iv2, iv3])

    if ulen > 0:
        Message(f"三弹区间: [{iv1[0]:.3f}, {iv1[1]:.3f}]  [{iv2[0]:.3f}, {iv2[1]:.3f}]  [{iv3[0]:.3f}, {iv3[1]:.3f}]", "DEBUG")
        Message(f"并集遮蔽时长={ulen:.3f}s, 方向={direction:.3f}rad, 速度={FY1_v:.3f}m/s, 投放tFly=({tFly1:.3f},{tFly2:.3f},{tFly3:.3f}), 延时tDrop=({tDrop1:.3f},{tDrop2:.3f},{tDrop3:.3f})", "INFO")
    return ulen

def update_best3():
    global maxTime3, direction_best, FY1_v_best, tFly1_best, tDrop1_best, tFly2_best, tDrop2_best, tFly3_best, tDrop3_best
    maxTime3 = nowTime3
    direction_best = direction
    FY1_v_best = FY1_v
    tFly1_best, tDrop1_best = tFly1, tDrop1
    tFly2_best, tDrop2_best = tFly2, tDrop2
    tFly3_best, tDrop3_best = tFly3, tDrop3
    Message(f"当前最优并集={maxTime3:.3f}s, dir={direction_best:.3f}, v={FY1_v_best:.3f}, "
            f"tFly=({tFly1_best:.3f},{tFly2_best:.3f},{tFly3_best:.3f}), "
            f"tDrop=({tDrop1_best:.3f},{tDrop2_best:.3f},{tDrop3_best:.3f})","INFO")

def update3(direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new):
    global direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3
    direction = direction_new
    FY1_v = FY1_v_new
    tFly1, tDrop1 = tFly1_new, tDrop1_new
    tFly2, tDrop2 = tFly2_new, tDrop2_new
    tFly3, tDrop3 = tFly3_new, tDrop3_new

def project_spacing(t1, t2, t3, delta=1.0):
    # 保证间隔 delta
    ts = sorted([t1, t2, t3])
    if ts[1] < ts[0] + delta:
        ts[1] = ts[0] + delta
    if ts[2] < ts[1] + delta:
        ts[2] = ts[1] + delta
    return ts[0], ts[1], ts[2]

def clamp_nonneg(x):
    return x if x >= 0.0 else 0.0

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def choose_new3(step):
    global direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3
    s0 = step * random.uniform(-1, 1)
    th1 = random.uniform(0, 2*math.pi)
    th2 = random.uniform(0, 2*math.pi)
    th3 = random.uniform(0, 2*math.pi)
    th4 = random.uniform(0, 2*math.pi)
    th5 = random.uniform(0, 2*math.pi)
    th6 = random.uniform(0, 2*math.pi)

    # 航向扰动
    direction_new = direction + s0 * math.sin(th1) * math.sin(th2) * math.pi
    if direction_new < 0: direction_new += 2*math.pi
    elif direction_new >= 2*math.pi: direction_new -= 2*math.pi

    # 速度扰动（保证区间为[70, 140]）
    FY1_v_new = clamp(FY1_v + s0 * math.sin(th1) * math.cos(th2) * 20.0, 70.0, 140.0)

    # 投放时刻扰动
    tFly1_new = tFly1 + s0 * math.cos(th1) * math.sin(th3) * 5.0
    tFly2_new = tFly2 + s0 * math.cos(th1) * math.cos(th3) * 5.0
    tFly3_new = tFly3 + s0 * math.cos(th1) * math.sin(th4) * 5.0

    tFly1_new, tFly2_new, tFly3_new = project_spacing(tFly1_new, tFly2_new, tFly3_new, delta=1.0)

    # 引信延时扰动
    tDrop1_new = clamp_nonneg(tDrop1 + s0 * math.cos(th1) * math.cos(th4) * 2.0)
    tDrop2_new = clamp_nonneg(tDrop2 + s0 * math.sin(th5) * math.sin(th4) * 2.0)
    tDrop3_new = clamp_nonneg(tDrop3 + s0 * math.cos(th6) * math.cos(th3) * 2.0)

    return [direction_new, FY1_v_new,tFly1_new, tDrop1_new,tFly2_new, tDrop2_new,tFly3_new, tDrop3_new]


Message("开始运行Problem3", "INFO")

real = Cylinder(r=7, height=10, x=0, y=200, z=0)
error = 1e-5
T = 2000.0
step = 5.0
alpha = math.exp(-1e-2)
INIT_MAX=60.0
drop_MAX=10.0

direction = random.uniform(0, 2*math.pi)
FY1_v = random.uniform(70, 140)

tFly1 = random.uniform(0, INIT_MAX-2.0)
tFly2 = tFly1 + 1.0 + random.uniform(0, 1.0)
tFly3 = tFly2 + 1.0 + random.uniform(0, 1.0)
tFly1, tFly2, tFly3 = project_spacing(tFly1, tFly2, tFly3, delta=1.0)

tDrop1 = random.uniform(0,drop_MAX)
tDrop2 = random.uniform(0,drop_MAX)
tDrop3 = random.uniform(0,drop_MAX)

nowTime3 = work_three(direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3)
attempts = 0
while nowTime3 < 0 and attempts < 1000:
    direction = random.uniform(0, 2*math.pi)
    FY1_v = random.uniform(70, 140)
    tFly1 = random.uniform(0, INIT_MAX-2.0)
    tFly2 = tFly1 + 1.0 + random.uniform(0, 1.0)
    tFly3 = tFly2 + 1.0 + random.uniform(0, 1.0)
    tFly1, tFly2, tFly3 = project_spacing(tFly1, tFly2, tFly3, delta=1.0)
    tDrop1 = random.uniform(0,drop_MAX)
    tDrop2 = random.uniform(0,drop_MAX)
    tDrop3 = random.uniform(0,drop_MAX)
    nowTime3 = work_three(direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3)
    attempts += 1

maxTime3 = -1.0
direction_best = FY1_v_best = 0.0
tFly1_best = tDrop1_best = tFly2_best = tDrop2_best = tFly3_best = tDrop3_best = 0.0

if nowTime3 > 0:
    update_best3()
else:
    Message("初始化失败（未找到可行遮蔽），请重试或调整参数", "WARNING")

Message(f"初始并集={maxTime3:.3f}s", "INFO")

for i in tqdm(range(3000)):
    direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new = choose_new3(step)
    predictTime = work_three(direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new)
    tries = 0
    while predictTime < 0 and tries < 50:
        direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new = choose_new3(step)
        predictTime = work_three(direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new)
        tries += 1

    if predictTime < 0:
        T *= alpha; step *= alpha
        if step <= error:
            break
        continue

    delta = predictTime - nowTime3
    if delta > 0 or math.exp(delta / T) > random.uniform(0, 1):
        update3(direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new)
        nowTime3 = predictTime
        if nowTime3 > maxTime3:
            update_best3()

    T *= alpha
    step *= alpha
    if step <= error:
        break

Message(f"最终最优并集={maxTime3:.3f}s, dir={direction_best:.3f}, v={FY1_v_best:.3f}, "
        f"tFly=({tFly1_best:.3f},{tFly2_best:.3f},{tFly3_best:.3f}), "
        f"tDrop=({tDrop1_best:.3f},{tDrop2_best:.3f},{tDrop3_best:.3f})", "INFO")
Message("运行结束Problem3","INFO")
