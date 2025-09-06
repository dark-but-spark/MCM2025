from modules.Fly import Fly
for i in range(1,4):
    f=open(f"data/FY{i}_gene.txt","a+")
    ans=[]
    for line in f:
        state=[float(x) for x in line.split()]
        ans.append(state)
    try:
        f2 = open(f"data/new_FY{i}_gene.txt", "r")
    except FileNotFoundError:
        continue
   
    for line in f2:
        new_state=[float(x) for x in line.split()]
        for state in ans:
            FL1=Fly(id=i,direction=state[0],v=state[1]).fly(state[2]).drop(state[3])
            FL2=Fly(id=i,direction=new_state[0],v=new_state[1]).fly(new_state[2]).drop(new_state[3])
            if (FL1.x-FL2.x)**2+(FL1.y-FL2.y)**2+(FL1.z-FL2.z)**2<100:
                break
        else:
            ans.append(new_state)
            f.write(f"{new_state[0]} {new_state[1]} {new_state[2]} {new_state[3]}\n")
    f2.close()
    f.close()