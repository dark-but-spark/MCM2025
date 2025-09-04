from modules.Cylinder import Cylinder

real= Cylinder(r=7,height=10,x=0,y=200,z=0)


checkN=1000
def check_visible():
    for i in range(checkN):
        point=real.random_point_on_surface()
        print(point)
        