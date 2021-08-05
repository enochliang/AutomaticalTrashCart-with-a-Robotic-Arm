import numpy as np
import GaussianElimination as GE

def getMatrix(Rx,Ry,Gx,Gy):
    rRx=0.0
    rRy=0.0
    rGx=rRx
    rGy=rRy+50.0
    A=np.array([[Rx,Ry,0,0],[0,0,Rx,Ry],[Gx,Gy,0,0],[0,0,Gx,Gy]])
    b=np.array([rRx,rRy,rGx,rGy])
    
    x=GE.GEPP(A,b)
    return x[0],x[1],x[2],x[3]