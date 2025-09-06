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

def work_interval(direction, FY1_v, FY1_tFly, FY1_tDrop, dt_coarse=0.13):
    """
    返回单枚干扰弹的遮蔽区间 [L, R]；若无遮蔽返回 (-1.0, -1.0)
    粗扫与二分均采用：重建对象，从 t0 连续 time_tick(...) 推进到目标时刻再判定。
    """
    FY1 = Fly(id=1, direction=direction, v=FY1_v).fly(FY1_tFly).drop(FY1_tDrop)
    t0  = FY1_tFly + FY1_tDrop
    tL, tR = t0, t0 + 20.0
    if tL >= tR: 
        return (-1.0, -1.0)

    def invisible_at(t_abs: float) -> bool:
        if t_abs < t0 or t_abs > tR: 
            return False
        M = Missile(id=1).time(t0)
        S = SmokeBall(x=FY1.x, y=FY1.y, z=FY1.z, r=10, v=3)
        dt = t_abs - t0
        M.time_tick(dt)
        S.time_tick(dt)
        return not Checker.visible(checkCylinder=real, smokeBall=S, missile=M)

    t = tL
    found = False
    while t <= tR + 1e-12:
        t += dt_coarse
        if invisible_at(t):
            LL = max(tL, t - dt_coarse)
            LR = min(tR, t)
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

    # 右端点二分
    RL, RR = L, tR
    while RR - RL > error:
        mid = 0.5 * (RL + RR)
        if invisible_at(mid):
            RL = mid
        else:
            RR = mid
    R = RL

    if R <= L + error:
        return (-1.0, -1.0)
    return (L, R)

def union_length(intervals):#区间并集
    ivs = [iv for iv in intervals if iv[0] >= 0.0 and iv[1] > iv[0] + error]
    if not ivs:#无有效区间
        return 0.0
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
               dt_coarse=0.13,
               overlap_penalty=0.2):
    """
    返回 union_len - λ*overlap,union_len
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

    #调试
    t0s = (tFly1_best + tDrop1_best, tFly2_best + tDrop2_best, tFly3_best + tDrop3_best)
    Message(f"[best t0] {t0s}", "INFO")
    iv1 = work_interval(direction_best, FY1_v_best, tFly1_best, tDrop1_best, dt_coarse=0.13)
    iv2 = work_interval(direction_best, FY1_v_best, tFly2_best, tDrop2_best, dt_coarse=0.13)
    iv3 = work_interval(direction_best, FY1_v_best, tFly3_best, tDrop3_best, dt_coarse=0.13)
    Message(f"[best ivs] {iv1} | {iv2} | {iv3}", "INFO")


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


def chain_seed_from_first(direction, v, tFly1, tDrop1, iv1, delta_gap=0.05):
    """
    已知首枚的遮蔽区间 iv1=[L,R] 在它附近构造第2、第3枚 尽量让三段相邻（小间隙）。
    返回：(tFly1,tDrop1,tFly2,tDrop2,tFly3,tDrop3)
    """
    L, R = iv1
    span = max(0.2, R - L)          # 首段时长（避免过小）
    t0_1 = tFly1 + tDrop1           # 首枚起爆时刻

    # 目标把三段排成： [L, R] 、 [R+gap, R+gap+span] 、 [R+2*gap, R+2*gap+span]
    t0_2_target = R + delta_gap
    t0_3_target = R + 2*delta_gap + span

    # 保持 tFly 两两至少 1s；简单取：
    tFly2 = max(0.0, tFly1 + 1.0)
    tFly3 = max(0.0, tFly2 + 1.0)

    # 反推对应的 tDrop，非负
    tDrop2 = max(0.0, t0_2_target - tFly2)
    tDrop3 = max(0.0, t0_3_target - tFly3)

    # 成对排序/间隔修正
    (tf1,td1),(tf2,td2),(tf3,td3) = enforce_spacing_pairs(
        [(tFly1,tDrop1),(tFly2,tDrop2),(tFly3,tDrop3)], delta=1.0
    )
    tDrop1 = max(0.0, t0_1 - tf1)
    tDrop2 = max(0.0, t0_2_target - tf2)
    tDrop3 = max(0.0, t0_3_target - tf3)
    return tf1, tDrop1, tf2, tDrop2, tf3, tDrop3


def choose_new(step):
    global direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3
    s0  = step * random.uniform(-1, 1)
    th1 = random.uniform(0, 2*math.pi)
    th2 = random.uniform(0, 2*math.pi)
    th3 = random.uniform(0, 2*math.pi)
    th4 = random.uniform(0, 2*math.pi)
    th5 = random.uniform(0, 2*math.pi)
    th6 = random.uniform(0, 2*math.pi)

    # 航向 & 速度
    direction_new = direction + s0 * math.sin(th1) * math.sin(th2) * math.pi
    if direction_new < 0: direction_new += 2*math.pi
    elif direction_new >= 2*math.pi: direction_new -= 2*math.pi
    FY1_v_new = clamp(FY1_v + s0 * math.sin(th1) * math.cos(th2) * 20.0, 70.0, 140.0)

    TF_MAX, TD_MAX = 180.0, 60.0

    # 以 t0 为主的扰动（让三段大体保持“接力”形态）
    t0_1 = tFly1 + tDrop1
    t0_2 = tFly2 + tDrop2
    t0_3 = tFly3 + tDrop3
    t0_1 += s0 * 0.8 * math.sin(th3)    # 主扰动：移动起爆时刻
    t0_2 += s0 * 0.8 * math.cos(th3)
    t0_3 += s0 * 0.8 * math.sin(th4)

    # 给 tFly 小幅扰动，保持 ≥0
    tFly1_new = clamp(tFly1 + s0 * 0.3 * math.sin(th5), 0.0, TF_MAX)
    tFly2_new = clamp(tFly2 + s0 * 0.3 * math.cos(th5), 0.0, TF_MAX)
    tFly3_new = clamp(tFly3 + s0 * 0.3 * math.sin(th6), 0.0, TF_MAX)

    # 由新的 t0 反推 tDrop（非负、上界）
    tDrop1_new = clamp(t0_1 - tFly1_new, 0.0, TD_MAX)
    tDrop2_new = clamp(t0_2 - tFly2_new, 0.0, TD_MAX)
    tDrop3_new = clamp(t0_3 - tFly3_new, 0.0, TD_MAX)

    # 成对排序 + ≥1s
    (tf1, td1), (tf2, td2), (tf3, td3) = enforce_spacing_pairs(
        [(tFly1_new, tDrop1_new), (tFly2_new, tDrop2_new), (tFly3_new, tDrop3_new)], delta=1.0
    )

    TD_MAX = 60.0
    tDrop1_new = clamp(t0_1 - tf1, 0.0, TD_MAX)
    tDrop2_new = clamp(t0_2 - tf2, 0.0, TD_MAX)
    tDrop3_new = clamp(t0_3 - tf3, 0.0, TD_MAX)
    return [direction_new, FY1_v_new, tf1, tDrop1_new, tf2, tDrop2_new, tf3, tDrop3_new]



def find_seed(INIT_MAX, DROP_INIT, dt_coarse=0.13):
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
                          dt_coarse=0.13,overlap_penalty=0.2)


while nowTime<0:
    direction=random.uniform(0,2*math.pi)
    FY1_v=random.uniform(70,140)
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
                          dt_coarse=0.13,overlap_penalty=0.2)
update_best()
# 自检打印 
ivs0 = [
        work_interval(direction, FY1_v, tFly1, tDrop1, dt_coarse=0.13),
        work_interval(direction, FY1_v, tFly2, tDrop2, dt_coarse=0.13),
        work_interval(direction, FY1_v, tFly3, tDrop3, dt_coarse=0.13),
        ]
Message(f"[init intervals] {ivs0}", "INFO")

# 避免初始的三枚导弹都是 (-1,-1)
if all(iv[0] < 0 for iv in ivs0):
    seed = find_seed(INIT_MAX, DROP_INIT, dt_coarse=0.13)
    if seed is not None:
        direction, FY1_v, tFly1, tDrop1, iv = seed
        tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3 = chain_seed_from_first(direction, FY1_v, tFly1, tDrop1, iv, delta_gap=0.05)
        nowscore, nowTime = work_three(direction, FY1_v,
                                   tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3,
                                   dt_coarse=0.13, overlap_penalty=0.2)
        Message(f"[seed used] dir={direction:.3f}, v={FY1_v:.1f}, iv={iv}, "
                f"tFly=({tFly1:.2f},{tFly2:.2f},{tFly3:.2f}), "
                f"tDrop=({tDrop1:.2f},{tDrop2:.2f},{tDrop3:.2f})", "INFO")
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
                                 dt_coarse=0.13,overlap_penalty=0.2)
    while predictScore < 0:
        direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new = choose_new(step)
        predictScore,predictTime = work_three(direction_new, FY1_v_new,
                                 tFly1_new, tDrop1_new,
                                 tFly2_new, tDrop2_new,
                                 tFly3_new, tDrop3_new,
                                 dt_coarse=0.13,overlap_penalty=0.2)
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
    MIN_STEP=1e-2
    if step <= MIN_STEP:
        step = MIN_STEP

Message(f"最终最优 score={maxscore:.3f}, Time={len_best:.3f} dir={direction_best:.3f}, v={FY1_v_best:.2f}, "
            f"tFly=({tFly1_best:.2f},{tFly2_best:.2f},{tFly3_best:.2f}), "
            f"tDrop=({tDrop1_best:.2f},{tDrop2_best:.2f},{tDrop3_best:.2f})", "INFO")
Message("运行结束 Problem3", "INFO")
