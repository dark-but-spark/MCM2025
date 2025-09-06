from util.Log import Message
from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile
from modules.Fly import Fly
import src.Checker as Checker
import math, random
from tqdm import tqdm

def enforce_spacing_pairs(pairs, delta=1.0):
    """
    pairs: [(tFly1,tDrop1), (tFly2,tDrop2), (tFly3,tDrop3)]
    - 按 tFly 升序排序
    - 钳到非负，并强制相邻投放时刻 ≥ delta
    - 返回三对(已按时间顺序对齐好的)新值
    """
    pairs = [(max(0.0, tf), max(0.0, td)) for (tf, td) in pairs]
    pairs.sort(key=lambda x: x[0])
    tf1, td1 = pairs[0]
    tf2, td2 = pairs[1]
    tf3, td3 = pairs[2]
    if tf2 < tf1 + delta:
        tf2 = tf1 + delta
    if tf3 < tf2 + delta:
        tf3 = tf2 + delta
    return (tf1, td1), (tf2, td2), (tf3, td3)

def work_interval(direction, FY1_v, FY1_tFly, FY1_tDrop, dt_coarse=0.2):
    """
    返回单枚干扰弹的遮蔽区间 [L, R]，若无遮蔽返回 (-1.0, -1.0)
    关键：粗扫和二分都使用“从 t0 连续 time_tick 到目标时刻再判定”
    """
    FY1 = Fly(id=1, direction=direction, v=FY1_v).fly(FY1_tFly).drop(FY1_tDrop)
    t0  = FY1_tFly + FY1_tDrop
    t_limit = t0 + 20.0

    # 统一的可见性判定：重建对象，从 t0 连续推进到 t_abs 后再判定
    def invisible_at(t_abs: float) -> bool:
        if t_abs < t0 - 1e-12:
            return False
        if t_abs > t_limit + 1e-12:
            return False  
        M = Missile(id=1).time(t_abs)
        S = SmokeBall(x=FY1.x, y=FY1.y, z=FY1.z, r=10, v=3).time(t_abs-FY1_tFly-FY1_tDrop)
        return not Checker.visible(checkCylinder=real, smokeBall=S, missile=M)

    # 粗扫：先推进，再判定
    t = t0
    found = False
    while t <= t_limit + 1e-12:
        t += dt_coarse
        M1 = Missile(id=1).time(t0)
        S1 = SmokeBall(x=FY1.x, y=FY1.y, z=FY1.z, r=10, v=3)
        M1.time_tick(t - t0)
        S1.time_tick(t - t0)
        if not Checker.visible(checkCylinder=real, smokeBall=S1, missile=M1):
            LL = max(t0, t - dt_coarse)  # 遮蔽开始在 (t - dt_coarse, t]
            LR = min(t_limit, t)
            found = True
            break

    if not found:
        return (-1.0, -1.0)

    # 左端点二分
    while LR - LL > error:
        mid = 0.5 * (LL + LR)
        if invisible_at(mid):
            LR = mid
        else:
            LL = mid
    L = LR

    #右端点二分 
    RL, RR = L, t_limit
    while RR - RL > error:
        mid = 0.5 * (RL + RR)
        if invisible_at(mid):
            RL = mid
        else:
            RR = mid
    R = RL

    if R <= L + error:
        return (-1.0, -1.0)
    
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
    return (L, R)

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

    (tf1, td1), (tf2, td2), (tf3, td3) = enforce_spacing_pairs([(tFly1, tDrop1), (tFly2, tDrop2), (tFly3, tDrop3)], delta=1.0)

    tFly1, tDrop1 = tf1, td1
    tFly2, tDrop2 = tf2, td2
    tFly3, tDrop3 = tf3, td3

    iv1 = work_interval(direction, FY1_v, tFly1, tDrop1, dt_coarse=dt_coarse)
    iv2 = work_interval(direction, FY1_v, tFly2, tDrop2, dt_coarse=dt_coarse)
    iv3 = work_interval(direction, FY1_v, tFly3, tDrop3, dt_coarse=dt_coarse)

    valid = [iv for iv in (iv1, iv2, iv3) if iv[0] >= 0.0]
    if not valid:
        return 0.0, 0.0

    union_len = union_length(valid)
    sum_len   = sum((b - a) for (a, b) in valid)
    overlap   = max(0.0, sum_len - union_len)
    score     = union_len - overlap_penalty * overlap
    return max(0.0, score), union_len

def update_best():
    global maxscore, direction_best, FY1_v_best, len_best
    global tFly1_best, tDrop1_best, tFly2_best, tDrop2_best, tFly3_best, tDrop3_best
    maxscore = nowscore
    len_best = nowTime
    direction_best = direction
    FY1_v_best = FY1_v
    tFly1_best, tDrop1_best = tFly1, tDrop1
    tFly2_best, tDrop2_best = tFly2, tDrop2
    tFly3_best, tDrop3_best = tFly3, tDrop3

    Message(f"[best] score={maxscore:.3f}, time={len_best:.3f},dir={direction_best:.3f}, v={FY1_v_best:.2f}, "
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

    tDrop1_new = clamp_nonneg(tDrop1 + s0 * math.cos(th1) * math.cos(th4) * 2.0)
    tDrop2_new = clamp_nonneg(tDrop2 + s0 * math.sin(th5) * math.sin(th4) * 2.0)
    tDrop3_new = clamp_nonneg(tDrop3 + s0 * math.cos(th6) * math.cos(th3) * 2.0)

    (tf1, td1), (tf2, td2), (tf3, td3) = enforce_spacing_pairs([(tFly1_new, tDrop1_new), (tFly2_new, tDrop2_new), (tFly3_new, tDrop3_new)], delta=1.0)
    tFly1_new, tDrop1_new = tf1, td1
    tFly2_new, tDrop2_new = tf2, td2
    tFly3_new, tDrop3_new = tf3, td3

    return [direction_new, FY1_v_new,
            tFly1_new, tDrop1_new,
            tFly2_new, tDrop2_new,
            tFly3_new, tDrop3_new]

def find_seed(INIT_MAX, DROP_INIT, dt_coarse=0.2):
    """
    粗网格扫一枚可行单弹，作为初始种子 避免三发都 (-1,-1)
    """
    dir_samples = 12
    v_grid      = [80, 100, 120, 140]
    t_grid      = [i * (INIT_MAX / 10.0) for i in range(10)]
    d_grid      = [j * (DROP_INIT / 10.0) for j in range(10)]
    for k in range(dir_samples):
        direction = 2*math.pi * k / dir_samples
        for v in v_grid:
            for tFly in t_grid:
                for tDrop in d_grid:
                    iv = work_interval(direction, v, tFly, tDrop, dt_coarse=dt_coarse)
                    if iv[0] >= 0.0:
                        return direction, v, tFly, tDrop, iv
    return None


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

tDrop1 = random.uniform(0.0, DROP_INIT)
tDrop2 = random.uniform(0.0, DROP_INIT)
tDrop3 = random.uniform(0.0, DROP_INIT)

(tf1, td1), (tf2, td2), (tf3, td3) = enforce_spacing_pairs([(tFly1, tDrop1), (tFly2, tDrop2), (tFly3, tDrop3)], delta=1.0)
tFly1, tDrop1 = tf1, td1
tFly2, tDrop2 = tf2, td2
tFly3, tDrop3 = tf3, td3


nowscore,nowTime = work_three(direction, FY1_v,
                          tFly1, tDrop1,
                          tFly2, tDrop2,
                          tFly3, tDrop3,
                          dt_coarse=0.2)
# 自检打印 
ivs0 = [
        work_interval(direction, FY1_v, tFly1, tDrop1, dt_coarse=0.2),
        work_interval(direction, FY1_v, tFly2, tDrop2, dt_coarse=0.2),
        work_interval(direction, FY1_v, tFly3, tDrop3, dt_coarse=0.2),
        ]
Message(f"[init intervals] {ivs0}", "INFO")

if all(iv[0] < 0 for iv in ivs0):
    seed = find_seed(INIT_MAX, DROP_INIT, dt_coarse=0.2)
    if seed is not None:
        direction, FY1_v, tFly1, tDrop1, iv = seed
        span = iv[1] - iv[0]
        # 其余两发在其后错开初始化
        tFly2 = tFly1 + 1.0 + max(2.0, span)
        tFly3 = tFly2 + 1.0 + max(2.0, span)
        tDrop2 = tDrop1
        tDrop3 = tDrop1
        nowscore,nowTime = work_three(direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3, dt_coarse=0.2)
        Message(f"[seed used] dir={direction:.3f}, v={FY1_v:.1f}, iv={iv}", "INFO")
    else:
        Message("种子扫描未找到单发遮蔽；建议再放宽 INIT_MAX / DROP_INIT 或减小 dt_coarse", "WARNING")


maxscore = -1.0
direction_best = FY1_v_best = 0.0
tFly1_best = tDrop1_best = tFly2_best = tDrop2_best = tFly3_best = tDrop3_best = 0.0

if nowscore > 0:
    update_best()
else:
    Message("初始化 score=0，进入退火搜索", "INFO")

T= 2000.0
alpha= math.exp(-1e-2)
step=5

for i in tqdm(range(3000)):
    direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new = choose_new(step)
    predictScore,predictTime= work_three(direction_new, FY1_v_new,
                                 tFly1_new, tDrop1_new,
                                 tFly2_new, tDrop2_new,
                                 tFly3_new, tDrop3_new,
                                 dt_coarse=0.2)
    while predictScore < 0:
        direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new = choose_new(step)
        predictScore,predictTime = work_three(direction_new, FY1_v_new,
                                 tFly1_new, tDrop1_new,
                                 tFly2_new, tDrop2_new,
                                 tFly3_new, tDrop3_new,
                                 dt_coarse=0.2)
    delta = predictScore - nowscore
    if delta > 0 or math.exp(delta / T) > random.random():
        update(direction_new, FY1_v_new,
                    tFly1_new, tDrop1_new,
                    tFly2_new, tDrop2_new,
                    tFly3_new, tDrop3_new)
        nowscore = predictScore
        nowTime = predictTime
        if nowscore> maxscore:
                update_best()

    T*= alpha
    step*= alpha
    if step <= error:
            break

Message(f"最终最优 score={maxscore:.3f}, Time={len_best:.3f} dir={direction_best:.3f}, v={FY1_v_best:.2f}, "
            f"tFly=({tFly1_best:.2f},{tFly2_best:.2f},{tFly3_best:.2f}), "
            f"tDrop=({tDrop1_best:.2f},{tDrop2_best:.2f},{tDrop3_best:.2f})", "INFO")
Message("运行结束 Problem3", "INFO")
