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

# === REPLACE: work_three，内部先转 t0 再映射到 (tFly,tDrop) ===
def work_three(direction, FY1_v,
               tFly1, tDrop1,
               tFly2, tDrop2,
               tFly3, tDrop3,
               dt_coarse=0.10,
               overlap_penalty=0.15):
    t01 = tFly1 + tDrop1
    t02 = tFly2 + tDrop2
    t03 = tFly3 + tDrop3
    (p1, p2, p3, _) = t0_to_release(t01, t02, t03)
    triples = [p1, p2, p3]

    # 先算一次，如果有无效段，做一次“接力修复”
    triples, ivs = repair_by_chaining(direction, FY1_v, triples, dt_coarse=dt_coarse, gap=0.08)

    valid = [iv for iv in ivs if iv[0] >= 0.0]
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

def t0_to_release(t01, t02, t03, TF_MAX=180.0, TD_MAX=60.0, eps=0.0):  # ← 0.2 -> 0.0
    tf1 = max(0.0, min(t01 - eps, TF_MAX)); td1 = t01 - tf1
    tf2 = max(0.0, min(t02 - eps, TF_MAX)); td2 = t02 - tf2
    tf3 = max(0.0, min(t03 - eps, TF_MAX)); td3 = t03 - tf3
    if tf2 < tf1 + 1.0:
        tf2 = tf1 + 1.0; t02 = max(t02, tf2 + eps); td2 = t02 - tf2
    if tf3 < tf2 + 1.0:
        tf3 = tf2 + 1.0; t03 = max(t03, tf3 + eps); td3 = t03 - tf3
    td1 = min(td1, TD_MAX); td2 = min(td2, TD_MAX); td3 = min(td3, TD_MAX)
    return (tf1, td1), (tf2, td2), (tf3, td3), (t01, t02, t03)

# 计算三段后，若有无效段，就把它们“接”在最近的有效段右端之后（修复函数）
def repair_by_chaining(direction, v, triples, dt_coarse=0.10, gap=0.08):
    # triples: [(tf1,td1),(tf2,td2),(tf3,td3)]
    ivs = [work_interval(direction, v, tf, td, dt_coarse=dt_coarse) for tf,td in triples]
    valid_idx = [i for i,iv in enumerate(ivs) if iv[0] >= 0.0]
    if len(valid_idx) == 3:
        return triples, ivs
    if not valid_idx:
        return triples, ivs  # 全无效，交回去让上层重采样

    # 以“最靠后的有效段”为锚点，把无效段往后接
    anchor = max(valid_idx, key=lambda i: ivs[i][1])
    R = ivs[anchor][1]

    tf_list = [tf for tf,_ in triples]
    # 目标起爆时刻按接力排：R+gap, R+2*gap (+给个最小 span 保护)
    min_span = max(0.5, ivs[anchor][1] - ivs[anchor][0]) if ivs[anchor][0] >= 0 else 1.0
    targets = []
    for k in range(3):
        if k == anchor: 
            targets.append(tf_list[k] + (triples[k][1]))  # 这个值不重要，只占位
        else:
            targets.append(R + gap)
            R += gap + min_span  # 下一段再往后一点

    # 把需要修复的段，按目标 t0 用 t0->(tFly,tDrop) 回推
    out = list(triples)
    for i in range(3):
        if i in valid_idx: 
            continue
        t0i = targets[i]
        (p1, p2, p3, _) = t0_to_release(
            t0i if i==0 else (out[0][0]+out[0][1]),
            t0i if i==1 else (out[1][0]+out[1][1]),
            t0i if i==2 else (out[2][0]+out[2][1]),
        )
        out = [p1, p2, p3]  # 简单起见，统一更新三段
        break  # 一次修一个，够用
    ivs2 = [work_interval(direction, v, tf, td, dt_coarse=dt_coarse) for tf,td in out]
    return out, ivs2

# === REPLACE: chain_seed_from_first，按 t0 接力，再统一映射 ===
def chain_seed_from_first(direction, v, tFly1, tDrop1, iv1, gap=0.10):
    L, R = iv1
    span = max(0.5, R - L)       # 第一段时长给个下限，避免太窄
    t01  = tFly1 + tDrop1
    t02  = R + gap
    t03  = R + 2*gap + span

    (p1, p2, p3, _) = t0_to_release(t01, t02, t03)  # p* 是 (tFly,tDrop) 对
    (tf1, td1), (tf2, td2), (tf3, td3) = p1, p2, p3
    return tf1, td1, tf2, td2, tf3, td3
# === REPLACE: choose_new_time_only ===
def choose_new_time_only(step):
    global direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3
    s0  = step * random.uniform(-1, 1)
    th1 = random.uniform(0, 2*math.pi)
    th2 = random.uniform(0, 2*math.pi)
    th3 = random.uniform(0, 2*math.pi)

    # 以 t0 为主的小扰动
    t01 = (tFly1 + tDrop1) + 0.8*s0*math.sin(th1)
    t02 = (tFly2 + tDrop2) + 0.8*s0*math.cos(th2)
    t03 = (tFly3 + tDrop3) + 0.8*s0*math.sin(th3)

    (p1, p2, p3, _) = t0_to_release(t01, t02, t03)
    (tf1, td1), (tf2, td2), (tf3, td3) = p1, p2, p3
    return [direction, FY1_v, tf1, td1, tf2, td2, tf3, td3]

# === REPLACE: choose_new_full（后20%再轻动方向与速度） ===
def choose_new_full(step):
    global direction, FY1_v, tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3
    s0  = step * random.uniform(-1, 1)
    th1 = random.uniform(0, 2*math.pi)
    th2 = random.uniform(0, 2*math.pi)
    th3 = random.uniform(0, 2*math.pi)

    # 轻微扰动航向与速度
    direction_new = direction + 0.12*s0*math.sin(th1)
    if direction_new < 0: direction_new += 2*math.pi
    elif direction_new >= 2*math.pi: direction_new -= 2*math.pi
    FY1_v_new = clamp(FY1_v + 1.2*s0*math.cos(th1), 70.0, 140.0)

    # 仍然主控 t0
    t01 = (tFly1 + tDrop1) + 0.8*s0*math.sin(th1)
    t02 = (tFly2 + tDrop2) + 0.8*s0*math.cos(th2)
    t03 = (tFly3 + tDrop3) + 0.8*s0*math.sin(th3)

    (p1, p2, p3, _) = t0_to_release(t01, t02, t03)
    (tf1, td1), (tf2, td2), (tf3, td3) = p1, p2, p3
    return [direction_new, FY1_v_new, tf1, td1, tf2, td2, tf3, td3]


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

# 先定义环境常量（work_interval 需要用到）
real  = Cylinder(r=7, height=10, x=0, y=200, z=0)
error = 1e-5

def single_seed(dt=0.06):
    """
    先大范围粗扫，再对最好的若干候选做一次局部精修（坐标搜索），
    尽量把单段做到 6s+。
    """
    candidates = []  # (span, d, v, tf, td, iv)

    # —— 粗扫：方向更密、tFly 更长 —— #
    for k in range(64):                     # 方向 64 等分（原来 16/32）
        d = 2*math.pi * k / 64
        for v in (70, 80, 90, 100, 110, 120, 130, 140):
            # tFly 扩到 0~120s，步长 3s（原来 0~40）
            for tf in range(0, 121, 3):
                # tDrop 更细：0.0、0.2、0.5~10.0（0.5 台阶）
                td_list = [0.0, 0.2] + [x/2 for x in range(1, 21)]  # 0.5..10.0
                for td in td_list:
                    iv = work_interval(d, v, float(tf), float(td), dt_coarse=dt)
                    if iv[0] >= 0.0:
                        span = iv[1] - iv[0]
                        candidates.append((span, d, v, float(tf), float(td), iv))

    if not candidates:
        return None

    # 取 Top-N 做局部精修
    candidates.sort(key=lambda x: x[0], reverse=True)
    topN = candidates[:12]   # 取前 12 个

    best = topN[0]
    for _, d0, v0, tf0, td0, iv0 in topN:
        span, d1, v1, tf1, td1, iv1 = refine_single(d0, v0, tf0, td0, iv0, dt_ref=0.05)
        if span > best[0]:
            best = (span, d1, v1, tf1, td1, iv1)
    return best
def refine_single(d, v, tf, td, iv_init, dt_ref=0.05):
    """
    对单发 (d, v, tf, td) 做几轮坐标搜索：依次微调 d/v/tf/td，
    只要能让区间变长就接受。返回更长的单段。
    """
    L0, R0 = iv_init
    best_span = R0 - L0
    best = (d, v, tf, td, (L0, R0))

    # 步长设定（可适当再减小）
    steps = {
        "d":  (0.06, 0.02, 8),     # 航向：先大后小，迭代 8 次
        "v":  (6.0,  2.0, 5),      # 速度：±6、±2
        "tf": (4.0,  1.0, 6),      # tFly：±4、±1
        "td": (1.0,  0.3, 6),      # tDrop：±1、±0.3
    }

    def try_update(d, v, tf, td):
        iv = work_interval(d, v, tf, td, dt_coarse=dt_ref)
        if iv[0] < 0.0:
            return None
        return (iv[1] - iv[0], d, v, tf, td, iv)

    # 坐标搜索：循环几轮
    for _ in range(3):
        improved = False
        # 依次优化四个变量
        for key in ("d", "v", "tf", "td"):
            big, small, times = steps[key]
            for step in (big, small):
                for _ in range(times):
                    d0, v0, tf0, td0, iv0 = best[0], best[1], best[2], best[3], best[4]
                    # 当前最优
                    d_cur, v_cur, tf_cur, td_cur, (L_cur, R_cur) = best[0], best[1], best[2], best[3], best[4]
                    span_cur = R_cur - L_cur

                    # 正负两个方向尝试
                    trials = []
                    if key == "d":
                        trials = [ (d_cur + step, v_cur, tf_cur, td_cur),
                                   (d_cur - step, v_cur, tf_cur, td_cur) ]
                    elif key == "v":
                        trials = [ (d_cur, max(70.0, min(140.0, v_cur + step)), tf_cur, td_cur),
                                   (d_cur, max(70.0, min(140.0, v_cur - step)), tf_cur, td_cur) ]
                    elif key == "tf":
                        trials = [ (d_cur, v_cur, max(0.0, tf_cur + step), td_cur),
                                   (d_cur, v_cur, max(0.0, tf_cur - step), td_cur) ]
                    else:  # td
                        trials = [ (d_cur, v_cur, tf_cur, max(0.0, td_cur + step)),
                                   (d_cur, v_cur, tf_cur, max(0.0, td_cur - step)) ]

                    better = None
                    for (dd, vv, tff, tdd) in trials:
                        res = try_update(dd, vv, tff, tdd)
                        if res and res[0] > span_cur + 1e-6:
                            if (better is None) or (res[0] > better[0]):
                                better = res
                    if better:
                        best_span, d, v, tf, td, iv = better
                        best = (d, v, tf, td, iv)
                        improved = True
                    else:
                        break  # 这一档步长没提升，换下一档或下一个变量
        if not improved:
            break

    d, v, tf, td, (L, R) = best
    return (R - L, d, v, tf, td, (L, R))


# 用单发种子 + 接力生成三发；如果找不到，再退回随机初始化
seed = single_seed(dt=0.10)
if seed:
    _, direction, FY1_v, tFly1, tDrop1, iv1 = seed
    
    # 接力构造 2/3 发
    best_pack = None
    for gap in (0.06,0.08, 0.10, 0.12, 0.14, 0.16):
        L, R = iv1
        span = max(0.5, R - L)
        # 用单段的右端 R 当第1段结束，接力排第2/3段的起爆时刻
        t02  = R + gap
        t03  = R + gap + span + gap

        (p1, p2, p3, _) = t0_to_release(tFly1 + tDrop1, t02, t03)
        (tf1, td1), (tf2, td2), (tf3, td3) = p1, p2, p3

        score, ulen = work_three(direction, FY1_v,
                                 tf1, td1, tf2, td2, tf3, td3,
                                 dt_coarse=0.09, overlap_penalty=0.15)
        if best_pack is None or score > best_pack[0]:
            best_pack = (score, ulen, tf1, td1, tf2, td2, tf3, td3, gap)

    # 用到目前最优的那组时间
    nowscore, nowTime = best_pack[0], best_pack[1]
    tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3 = best_pack[2:8]
    Message(f"[seed-chain] gap={best_pack[8]:.2f}, score={nowscore:.3f}, time={nowTime:.3f}", "INFO")
else:
    Message("粗搜不到单发遮蔽，转为随机初始化", "WARNING")
    direction = random.uniform(0, 2*math.pi)
    FY1_v     = random.uniform(70, 140)
    INIT_MAX, DROP_INIT = 180.0, 60.0
    tFly1 = random.uniform(0.0, INIT_MAX - 2.0)
    tFly2 = tFly1 + 1.0 + random.uniform(0.0, 1.0)
    tFly3 = tFly2 + 1.0 + random.uniform(0.0, 1.0)
    tDrop1 = random.uniform(0.0, DROP_INIT)
    tDrop2 = random.uniform(0.0, DROP_INIT)
    tDrop3 = random.uniform(0.0, DROP_INIT)
    (tf1, td1), (tf2, td2), (tf3, td3) = enforce_spacing_pairs(
        [(tFly1, tDrop1), (tFly2, tDrop2), (tFly3, tDrop3)], delta=1.0)
    tFly1, tDrop1 = tf1, td1
    tFly2, tDrop2 = tf2, td2
    tFly3, tDrop3 = tf3, td3

# —— 计算初值得分（建议用 dt_coarse=0.10, overlap_penalty=0.15）——
nowscore, nowTime = work_three(
    direction, FY1_v,
    tFly1, tDrop1, tFly2, tDrop2, tFly3, tDrop3,
    dt_coarse=0.10, overlap_penalty=0.15
)
update_best()

# 自检打印
ivs0 = [
    work_interval(direction, FY1_v, tFly1, tDrop1, dt_coarse=0.10),
    work_interval(direction, FY1_v, tFly2, tDrop2, dt_coarse=0.10),
    work_interval(direction, FY1_v, tFly3, tDrop3, dt_coarse=0.10),
]
Message(f"[init intervals] {ivs0}", "INFO")

# 退火参数
T      = 1200.0
alpha  = math.exp(-8e-3)
step   = 4.0
N_iter = 3000

for it in tqdm(range(N_iter)):
    # 前 80% 只调时间；后 20% 再轻动方向/速度
    cand = choose_new_time_only(step) if it < 0.8*N_iter else choose_new_full(step)

    direction_new, FY1_v_new, tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new = cand
    predictScore, predictTime = work_three(
        direction_new, FY1_v_new,
        tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new,
        dt_coarse=0.10, overlap_penalty=0.15
    )

    delta = predictScore - nowscore
    if delta > 0 or math.exp(delta / T) > random.random():
        update(direction_new, FY1_v_new,
               tFly1_new, tDrop1_new, tFly2_new, tDrop2_new, tFly3_new, tDrop3_new)
        nowscore, nowTime = predictScore, predictTime
        if nowscore > maxscore:
            update_best()

    T    *= alpha
    step *= alpha
    if step < 0.02:
        step = 0.02

Message(
    f"最终最优 score={maxscore:.3f}, Time={len_best:.3f} "
    f"dir={direction_best:.3f}, v={FY1_v_best:.2f}, "
    f"tFly=({tFly1_best:.2f},{tFly2_best:.2f},{tFly3_best:.2f}), "
    f"tDrop=({tDrop1_best:.2f},{tDrop2_best:.2f},{tDrop3_best:.2f})",
    "INFO"
)
Message("运行结束 Problem3", "INFO")
