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
from concurrent.futures import ThreadPoolExecutor, as_completed
import os


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
def solve(state):
    intervals=[]
    for i in range(0, len(state), 4):
        direction=state[i]
        FY1_v=state[i+1]
        FY1_tFly=state[i+2]
        FY1_tDrop=state[i+3]
        time=work(direction,FY1_v,FY1_tFly,FY1_tDrop,1,i//4+1)
        if time!=-1:
            intervals.append(time)
    if len(intervals)==0:
        return -1
    intervals = sorted(intervals, key=lambda x: (x[0]))
    ans=intervals[0][1]-intervals[0][0]
    R=intervals[0][1]
    for i in range(1,len(intervals)):
        if intervals[i][1]>R:
            if intervals[i][0]>R:
                ans+=intervals[i][1]-intervals[i][0]
                R=intervals[i][1]
            else:
                ans+=intervals[i][1]-R
                R=intervals[i][1]
    return ans
            
def update_best():
    global Time_best, state_best, Time_personal_best, state_personal_best
    for i in range(N):
        if Time[i]>Time_best:
            Time_best=Time[i]
            state_best=state[i][:]
            Message(f"目前最优解，烟幕保护时间为{Time_best:.3f}s，状态为{state_best}", "INFO")
        if Time[i]>Time_personal_best[i] or Time_personal_best[i]==0:
            Time_personal_best[i]=Time[i]
            state_personal_best[i]=state[i][:]

    
def update(state_new,v_new,Time_new,i):
    global state,v,Time
    state[i] = state_new[:]
    v[i] = v_new[:]
    Time[i] = Time_new


def new_v(m,i):
    '''
    更新速度
    m: 迭代次数
    i: 粒子编号
    '''
    vv=deepcopy(v[i])
    w=w_max-(w_max-w_min)*m/M
    c1=3*w-0.5
    c2=-3*w+3
    if random.uniform(0,1)<0.1*math.log(1+m/M)/0.7:
        c1=0 #自我粒子
    elif random.uniform(0,1)<0.1*math.log(2-m/M)/0.7:
        c2=0 #社会粒子
    for j in range(n):
        r1=random.uniform(0,1)
        r2=random.uniform(0,1)
        vv[j]=w*vv[j]+c1*r1*(state_personal_best[i][j]-state[i][j])+c2*r2*(state_best[j]-state[i][j])
    return vv
def new_state(i,v_new):
    '''
    更新粒子i的位置
    '''
    state_new=deepcopy(state[i])
    for j in range(0,n,4):
        state_new[j]=(state[i][j]+v_new[j]+2*math.pi)%(2*math.pi)
        state_new[j+1]=(state[i][j+1]+v_new[j+1]+70)%70+70
        state_new[j+2]=(state[i][j+2]+v_new[j+2]+35)%35
        state_new[j+3]=(state[i][j+3]+v_new[j+3]+35)%35
    return state_new

def init_state():
    new_state=[0]*n 
    for i in range(0,n,4):
        new_state[i]=random.uniform(0,2*math.pi)
        new_state[i+1]=random.uniform(70,140)
        new_state[i+2]=random.uniform(0,35)
        new_state[i+3]=random.uniform(0,35-new_state[i+2])
    return new_state
def init_v():
    new_v=[0]*n 
    for i in range(0,n,4):
        new_v[i]=random.uniform(-2*math.pi*0.10,2*math.pi*0.10)
        new_v[i+1]=random.uniform(-35*0.10,35*0.10)
        new_v[i+2]=random.uniform(-35*0.10,35*0.10 )
        new_v[i+3]=random.uniform(-35*0.10,35*0.10)
    return new_v


real= Cylinder(r=7,height=10,x=0,y=200,z=0)
error=1e-5

N=50#粒子数量
M=1000#最大迭代次数
n=12
w_max=0.9;w_min=0.4
state=[[] for _ in range(N)]# 记录状态
# 分别为 FY1的direction，speed，fly时间，drop时间 FY2的direction，speed，fly时间，drop时间 FY3的direction，speed，fly时间，drop时间
v=[[] for _ in range(N)]
Time=[0]*N
state_best=[[] for _ in range(N)]
Time_best=0
state_personal_best=[[] for _ in range(N)]
Time_personal_best=[0]*N


# 多线程并发初始化所有粒子的状态和时间

# --- 批量分包工具函数 ---
def chunk_indices(N, num_chunks):
    chunk_size = (N + num_chunks - 1) // num_chunks
    return [(i * chunk_size, min((i + 1) * chunk_size, N)) for i in range(num_chunks)]

def batch_init_states(indices):
    batch_states = []
    batch_times = []
    batch_vs = []
    for idx in tqdm(range(*indices)):
        state_new = init_state()
        nowTime = solve(state_new)
        while nowTime < 0:
            state_new = init_state()
            nowTime = solve(state_new)
        v_new = init_v()
        batch_states.append(state_new[:]) 
        batch_times.append(nowTime)
        batch_vs.append(v_new)
    return batch_states, batch_times, batch_vs




def batch_solve(states_batch):
    return [solve(state) for state in states_batch]


def hybrid(state, v):
    i=random.randint(0,N-1)
    j=random.randint(0,N-1)
    k=random.randint(0,2)
    for _ in range(k*4,(k+1)*4):
        temp=state[i][_]
        state[i][_] = state[j][_]
        state[j][_] = temp
        temp=v[i][_]
        v[i][_] = v[j][_]
        v[j][_] = temp

def main():
    global Time_best, state_best, Time_personal_best, state_personal_best
    Message("开始运行Problem4","INFO")
    num_chunks = max(1, os.cpu_count()-2)
    indices_list = chunk_indices(N, num_chunks)
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=num_chunks) as executor:
        results = list(executor.map(batch_init_states, indices_list))
        # 展开结果
        idx = 0
        for batch_states, batch_times, batch_vs in results:
            for s, t, v_new in zip(batch_states, batch_times, batch_vs):
                state[idx] = s
                Time[idx] = t
                v[idx] = v_new
                idx += 1
    update_best()
    
    for i in tqdm(range(M)):
        # 构造所有新状态   
        if i%2==1:
            v_news = [new_v(i, j) for j in range(N)]
            states_new = [new_state(j, v_news[j]) for j in range(N)]
        else:
            v_news = [deepcopy(v[j]) for j in range(N)]
            states_new = [deepcopy(state[j]) for j in range(N)]
            hybrid_time=random.randint(10,N//4)
            for _ in range(hybrid_time):
                hybrid(states_new, v_news)
        # 分包
        state_batches = [states_new[start:end] for start, end in indices_list]
        with ProcessPoolExecutor(max_workers=num_chunks) as executor:
            results = list(executor.map(batch_solve, state_batches))
            nowTimes = [item if item >= 0 else 0 for batch in results for item in batch]
        # 更新所有粒子
        for j in range(N):
            update(states_new[j], v_news[j], nowTimes[j], j)
        update_best()

    Message(f"目前最优解，烟幕保护时间为{Time_best:.3f}s，状态为{state_best}", "INFO")
    Message("运行结束Problem4","INFO")
      
if __name__ == "__main__":
    main()