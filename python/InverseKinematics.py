import numpy as np
import math


a=47.5
b=109
L1=a+b
L2=260


def InverseAlgorithm(X,Y,Z,a=47.5,b=109,L1=a+b,L2=260):    #mm

    Rsq=(X*X)+(Y*Y)+(Z*Z)    #原點到夾爪距離

    A=0.0                                               #定義第一關節角度變數

    C=-math.acos((Rsq-(L2*L2)-(L1*L1))/(2*L1*L2))       #計算第三關節角度
    
    B=math.asin(-Y/(L2*math.sin(C)))                    #計算第二關節角度 需討論範圍

    if(Z>=0):
        B=math.pi-B

    f1=(L2*math.cos(C))+b
    f2=-(L2*math.sin(C))
    
    g1=f1+a
    g3=-(math.cos(B)*f2)


    D=4*((g3*g3)+(g1*g1)-(Z*Z))   			     #D判別式(由Z項討論)
    U=[0.0,0.0]
    U[0]=(2*g1+math.sqrt(D))/((Z+g3)*2)                      #求U兩根
    U[1]=(2*g1-math.sqrt(D))/((Z+g3)*2)
    
    

    E=4*((g3*g3)+(g1*g1)-(X*X))   			     #E判別式(由X項討論)
    W=[0.0,0.0]
    W[0]=((-2)*g3+math.sqrt(E))/((X+g1)*2)                   #求W兩根
    W[1]=((-2)*g3-math.sqrt(E))/((X+g1)*2)

    
    
    for i in range(2):                                       #判斷U,W的哪兩根相同就為實際解
        for j in range(2):
            if(abs(U[i]-W[j])<0.001):
                A=2*math.atan(W[j])

#########三關節角度##########
    TH=[180*A/math.pi,180*B/math.pi,180*C/math.pi]

    
    return TH
