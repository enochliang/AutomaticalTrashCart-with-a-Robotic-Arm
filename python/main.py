#引用模塊
import numpy as np
import serial as sl
import time
import struct
import PathCompute as PC
import InverseKinematics as IK
import positioning as PS
import coordination as CO
import contour
import object_detection_yolo as test

#Serial port 建立連線
ser = sl.Serial('COM6', 115200, timeout=3)



#打包float資料函數(Packs a python 4 byte float to an arduino float)
def packIntegerAsFloat(value):         
    return struct.pack('<f', value)

def SendChar(x):
    toSend=struct.pack('<c', bytes(x,"utf-8"))
    ser.write(toSend)




#===========主函式==========
def main():
    
    
    
    Angle0=[90.0,180.0,-90.0,0.0]     #初始角度
    Anglef=[0.0,0.0,0.0,0.0]
    coef=np.array([[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]])     #係數矩陣
    #Center=[0.0,0.0]
    Ccode=0   #Confirmation_Code
    Sendcode=0
    
    
    #鏡頭定位
    Rx,Ry,Gx,Gy=PS.FindCenter()
    a,b,c,d,Orix,Oriy=CO.getMatrix(Rx,Ry,Gx,Gy)
    Matrix=np.array([[a,b],[c,d]])
    print("a=",a,"b=",b,"c=",c,"d=",d)
    
    #發送訊號啟動自走車
    
    
    #接收自走車到達終點訊號
    

    #手臂任務
    while (True):
        tX=None
        tY=None
        tZ=None

        
        CLASS=None
        CLASS,YoloX,YoloY=test.YOLOdetect()
        print('a')
        print(CLASS)
        
        #取得物體XYA
        fX,fY,fA=contour.FindContour()
        if(abs(YoloX-fX)>50 or abs(YoloY-fY)>50):
            continue
        #像素中的向量
        Vec=np.array([(fX-Orix),(fY-Oriy)])
        X=float(np.dot(Matrix[0],Vec))
        Y=float(np.dot(Matrix[1],Vec))
##        if((X<100.0)|(X>350.0)):
##            print("a")
##            continue
##        if((Y<-50.0)|(Y>50.0)):
##            print("b")
##            continue
        if(Y<0.0):
            Y=0.0
        #設定物體Z
        Z=-150.0
        A=fA
        if(0<=A<=90):
            A-=90
        elif(A<0):
            A+=90
        print("X=",X,"Y=",Y,"Z=",Z,"A=",A,"Ox=",Orix,"Oy=",Oriy)
        #輸入目標座標
##        X=float(input('X='))
##        Y=float(input('Y='))
##        Z=float(input('Z='))
##        A=float(input('A='))

        
        #----------------1.始位置 伸到 物體上--------------
        #用目標座標計算目標姿態
        Angle=IK.InverseAlgorithm(X,Y,Z+120.0)
        for i in range(3):
            Anglef[i]=Angle[i]
        Anglef[3]=A#改F(A)

        #計算馬達運動軌跡係數
        for i in range(4):
            for j in range(4):
                coef[i][j]=PC.ARM_curve(Angle0[i], Anglef[i])[j].item()


        ser.flushOutput()
        #發送係數矩陣
        for i in range(4):
            for j in range(4):
                toSEND=packIntegerAsFloat(coef[i][j].item())
                ser.write(toSEND)

        
        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")
            
        print("1A")

        ser.flushOutput()
        #夾爪開
        SendChar("O")

        print("1B")
        
        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting):
                Ccode=ser.read()
                break

        print("1C")
        
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")
        
        #將本次目標姿態處存為下次初始姿態
        for i in range(4):
            Angle0[i]=Anglef[i]


        print("1K")

        #----------------2.物體上 伸到 物體 --並且夾取--------------
        #用目標座標計算目標姿態
        Angle=IK.InverseAlgorithm(X,Y,Z)
        for i in range(3):
            Anglef[i]=Angle[i]
        Anglef[3]=A#改F(A)

        #計算馬達運動軌跡係數
        for i in range(4):
            for j in range(4):
                coef[i][j]=PC.ARM_curve(Angle0[i], Anglef[i])[j].item()


        ser.flushOutput()
        #發送係數矩陣
        for i in range(4):
            for j in range(4):
                toSEND=packIntegerAsFloat(coef[i][j].item())
                ser.write(toSEND)

        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        print("2A")

        ser.flushOutput()
        #夾爪閉
        SendChar("C")

        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        #將本次目標姿態處存為下次初始姿態
        for i in range(4):
            Angle0[i]=Anglef[i]


        print("2K")
        #----------------3.將物體提起----------------
        #用目標座標計算目標姿態
        Angle=IK.InverseAlgorithm(X,Y,Z+120.0)
        for i in range(3):
            Anglef[i]=Angle[i]
        Anglef[3]=A

        #計算馬達運動軌跡係數
        for i in range(4):
            for j in range(4):
                coef[i][j]=PC.ARM_curve(Angle0[i], Anglef[i])[j].item()


        ser.flushOutput()
        #發送係數矩陣
        for i in range(4):
            for j in range(4):
                toSEND=packIntegerAsFloat(coef[i][j].item())
                ser.write(toSEND)


        print("err")
        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        print("3A")

        ser.flushOutput()
        #夾爪閉
        SendChar("C")

        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        #將本次目標姿態處存為下次初始姿態
        for i in range(4):
            Angle0[i]=Anglef[i]

        print("3K")
##        #----------------4.將物體舉至鏡頭下----------------
##        
##
##        #換算鏡頭下中心座標
##        CVec=np.array([240.0,320.0])
##        CX=float(np.dot(Matrix[0],CVec))
##        CY=float(np.dot(Matrix[1],CVec))
##        if(CY<0.0):
##            CY=0.0
##
##        #用目標座標計算目標姿態
##        Angle=IK.InverseAlgorithm(CX-50.0,CY,290.0)
##        for i in range(3):
##            Anglef[i]=Angle[i]
##        Anglef[3]=90.0
##
##        #計算馬達運動軌跡係數
##        for i in range(4):
##            for j in range(4):
##                coef[i][j]=PC.ARM_curve(Angle0[i], Anglef[i])[j].item()
##
##
##        ser.flushOutput()
##        #發送係數矩陣
##        for i in range(4):
##            for j in range(4):
##                toSEND=packIntegerAsFloat(coef[i][j].item())
##                ser.write(toSEND)
##
##        #接收Arduino傳來確認碼Done
##        while(1):
##            if(ser.in_waiting>0):
##                Ccode=ser.read()
##                break
##        Ccode=bytes.decode(Ccode)
##        if(Ccode!="D"):
##            print("error")
##
##        
##
##
##        ser.flushOutput()
##        #夾爪閉
##        SendChar("C")
##
##        #接收Arduino傳來確認碼Done
##        while(1):
##            if(ser.in_waiting>0):
##                Ccode=ser.read()
##                break
##        Ccode=bytes.decode(Ccode)
##        if(Ccode!="D"):
##            print("error")
##
##        #將本次目標姿態處存為下次初始姿態
##        for i in range(4):
##            Angle0[i]=Anglef[i]
##
##        time.sleep(10)
##        print("4K")


        #----------------5.從鏡頭下夾到繞道點----------------
        #用目標座標計算目標姿態
        Angle=IK.InverseAlgorithm(60.0,250.0,140.0)
        for i in range(3):
            Anglef[i]=Angle[i]
        Anglef[3]=90.0

        #計算馬達運動軌跡係數
        for i in range(4):
            for j in range(4):
                coef[i][j]=PC.ARM_curve(Angle0[i], Anglef[i])[j].item()


        ser.flushOutput()
        #發送係數矩陣
        for i in range(4):
            for j in range(4):
                toSEND=packIntegerAsFloat(coef[i][j].item())
                ser.write(toSEND)

        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        print("4A")


        ser.flushOutput()
        #夾爪閉
        SendChar("C")

        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        #將本次目標姿態處存為下次初始姿態
        for i in range(4):
            Angle0[i]=Anglef[i]

        print("5K")
        #----------------6.將物體丟垃圾桶----------------
        #判斷種類
        if(CLASS==0):#Retoro
            tX=-250.0
            tY=0.0
            tZ=150.0
        elif(CLASS==1):#Tin
            tX=-350.0
            tY=0.0
            tZ=150.0
        elif(CLASS==2):#Plastic
            tX=-150.0
            tY=0.0
            tZ=150.0
        #用目標座標計算目標姿態
        Angle=IK.InverseAlgorithm(tX,tY,tZ+50.0)
        for i in range(3):
            Anglef[i]=Angle[i]
        Anglef[3]=0.0

        #計算馬達運動軌跡係數
        for i in range(4):
            for j in range(4):
                coef[i][j]=PC.ARM_curve(Angle0[i], Anglef[i])[j].item()


        ser.flushOutput()
        #發送係數矩陣
        for i in range(4):
            for j in range(4):
                toSEND=packIntegerAsFloat(coef[i][j].item())
                ser.write(toSEND)

        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        print("5A")

        ser.flushOutput()
        #夾爪閉
        SendChar("O")

        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        #將本次目標姿態處存為下次初始姿態
        for i in range(4):
            Angle0[i]=Anglef[i]


        print("6K")
        #----------------7.回歸初始位置----------------
        #用目標座標計算目標姿態
        
        Angle=IK.InverseAlgorithm(-230.0,0.0,156.5)
        for i in range(3):
            Anglef[i]=Angle[i]
        Anglef[3]=0.0

        #計算馬達運動軌跡係數
        for i in range(4):
            for j in range(4):
                coef[i][j]=PC.ARM_curve(Angle0[i], Anglef[i])[j].item()


        ser.flushOutput()
        #發送係數矩陣
        for i in range(4):
            for j in range(4):
                toSEND=packIntegerAsFloat(coef[i][j].item())
                ser.write(toSEND)

        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        print("6A")

        ser.flushOutput()
        #夾爪閉
        SendChar("C")

        #接收Arduino傳來確認碼Done
        while(1):
            if(ser.in_waiting>0):
                Ccode=ser.read()
                break
        Ccode=bytes.decode(Ccode)
        if(Ccode!="D"):
            print("error")

        #將本次目標姿態處存為下次初始姿態
        for i in range(4):
            Angle0[i]=Anglef[i]

        print("7K")
        
    
    
    



if __name__ == "__main__":
    main()
