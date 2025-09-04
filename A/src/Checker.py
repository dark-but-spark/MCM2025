from modules.Cylinder import Cylinder
from modules.SmokeBall import SmokeBall
from modules.Missile import Missile

def cross(smokeBall, A, M):
    '''
    检测烟雾弹有没有让M找不到checkPoint
        
    Args:
        smokeBall: SmokeBall object 烟雾弹
        A: (x,y,z) tuple 目标点
        M: M object 导弹
    Returns:
        True or False 找没找到
    '''
    import math
    import numpy as np
    AM=(np.array(A)-np.array([M.x,M.y,M.z]))
    AO=(np.array([smokeBall.x,smokeBall.y,smokeBall.z])-np.array(A))
    cosOAM=np.dot(AM,AO)/(math.sqrt(np.dot(AM,AM))*math.sqrt(np.dot(AO,AO)))
    d=math.sqrt(np.dot(AO,AO))*math.sqrt(1-cosOAM**2)
    l=math.sqrt(np.dot(AO,AO))*cosOAM
    if d<smokeBall.r and math.sqrt(np.dot(AM,AM))>l-(smokeBall.r**2-d**2)**0.5:
        return True
    else:
        return False
    
def visible(checkCylinder,smokeBall,missile):
    '''
    检测导弹能不能看到真目标
    Args:
        checkCylinder: Cylinder object 真目标
        smokeBall: SmokeBall object 烟雾弹
        missile: Missile object 导弹
    Returns:
        True or False 能看到真目标 or 不能看到真目标
    '''
    checkN=1000
    for i in range(checkN):
        point=checkCylinder.random_point_on_surface()
        if not cross(smokeBall=smokeBall, A=point, M=missile):
            return True
    return False