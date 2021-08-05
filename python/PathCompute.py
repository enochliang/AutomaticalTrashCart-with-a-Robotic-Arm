import numpy as np
import GaussianElimination as GE

def ARM_curve(Angle0,Anglef):##"""Provide initial Angle and final Angle to obtain the coefficient of the cube polynomial."""
    T=150.0
    A=np.array([[0.0,0.0,0.0,1.0],[T*T*T,T*T,T,1.0],[0.0,0.0,1.0,0.0],[3.*T*T,2.*T,1.0,0.0]])
    b=np.array([[Angle0],[Anglef],[0.0],[0.0]])
    return GE.GEPP(A, b)
