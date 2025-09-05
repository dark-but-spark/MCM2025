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
    AM=(np.array([M.x,M.y,M.z])-np.array(A))
    AO=(np.array([smokeBall.x,smokeBall.y,smokeBall.z])-np.array(A))
    if np.dot(AO,AO)<=smokeBall.r**2:
        return True
    # if np.dot(AM,AO)<=0:
    #     return False
    cosOAM=np.dot(AM,AO)/(math.sqrt(np.dot(AM,AM))*math.sqrt(np.dot(AO,AO)))
    d=math.sqrt(np.dot(AO,AO))*math.sqrt(1-cosOAM**2)
    l=math.sqrt(np.dot(AO,AO))*cosOAM
    if d<=smokeBall.r and math.sqrt(np.dot(AM,AM))>=l-(smokeBall.r**2-d**2)**0.5:
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
    import math
    import numpy as np
    OM=(np.array([missile.x,missile.y,missile.z])-np.array([smokeBall.x,smokeBall.y,smokeBall.z]))
    if np.dot(OM,OM)<=smokeBall.r**2:
        return False# 导弹在烟雾弹内，必然看不到真目标
    if missile.x<0:
        return True# 导弹已经炸了
    checkN=50
    for i in range(checkN):
        point=checkCylinder.random_point_on_surface()
        if not cross(smokeBall=smokeBall, A=point, M=missile):
            return True
    return False