from modules.Fly import Fly
for i in range(1,6):
    f=open(f"data/FY{i}_genePro.txt","a+")
    ans=[]
    for line in f:
        state=[float(x) for x in line.split()]
        ans.append(state)
    try:
        f2 = open(f"data/new_FY{i}_genePro.txt", "r")
    except FileNotFoundError:
        continue 
    for line in f2:
        new_state=[float(x) for x in line.split()]
        for state in ans:
            for j in range(3):
                FL1=Fly(id=i+1,direction=state[0],v=state[1]).fly(state[2*j+2]).drop(state[2*j+3])
                FL2=Fly(id=i+1,direction=new_state[0],v=new_state[1]).fly(new_state[2*j+2]).drop(new_state[2*j+3])
                if (FL1.x-FL2.x)**2+(FL1.y-FL2.y)**2+(FL1.z-FL2.z)**2>100:
                    break
            else:
                break
        else:
            ans.append(new_state)
            f.write(f"{new_state[0]} {new_state[1]} {new_state[2]} {new_state[3]} {new_state[4]} {new_state[5]} {new_state[6]} {new_state[7]} {new_state[8]}\n")
    f2.close()
    f.close()