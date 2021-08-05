import numpy as np
import GaussianElimination as GE
import math

def getMatrix(Rx,Ry,Gx,Gy):
##    rRx=50.0
##    rRy=50.0
##    rGx=rRx
##    rGy=rRy+50.0


    R=math.sqrt((Gx-Rx)*(Gx-Rx)+(Gy-Ry)*(Gy-Ry))
    Theta=math.atan((Gy-Ry)/(Gx-Rx))
##    if (Theta<0):
##        Theta=Theta+(math.pi)

    #(0,0)的像素座標
    Orix=int(Rx-R*math.sin(Theta)-R*math.cos(Theta))
    Oriy=int(Ry-R*math.sin(Theta)+R*math.cos(Theta))

    #(50,0)的像素座標
    X50x=int(Orix+R*math.cos(Theta))
    X50y=int(Orix-R*math.sin(Theta))

    #(0,50)的像素座標
    X05x=int(Orix-R*math.sin(Theta))
    X05y=int(Orix+R*math.cos(Theta))
    
    A=np.array([[(X50x-Orix),(X50y-Oriy),0,0],[0,0,(X50x-Orix),(X50y-Oriy)],[(X05x-Orix),(X05y-Oriy),0,0],[0,0,(X05x-Orix),(X05y-Oriy)]])
    b=np.array([50.0,0.0,0.0,50.0])
    
    x=GE.GEPP(A,b)
    return x[0],x[1],x[2],x[3],Orix,Oriy
