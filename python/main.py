#引用模塊
import numpy as np
import serial as sl
import time
import struct
import PathCompute as PC
import InverseKinematics as IK
import positioning as PS
import coordination as CO

#Serial port 建立連線
ser = sl.Serial('COM6', 115200, timeout=3)



#打包float資料函數(Packs a python 4 byte float to an arduino float)
def packIntegerAsFloat(value):         
    return struct.pack('<f', value)

def SendChar(x):
    toSend=struct.pack('<c', bytes(x,"utf-8"))
    ser.write(toSend)

#攝影機像素對公分座標轉換
def Coor(X,Y,a,b,c,d):
    A=np.array([[a,b],[c,d]])
    x=np.array([X,Y])
    y=np.array([np.dot(A[0],x),np.dot(A[1],x)])


#===========主函式==========
def main():
    
    
    
    Angle0=[90.0,180.0,-90.0,0.0]     #初始角度
    Anglef=[0.0,0.0,0.0,0.0]
    coef=np.array([[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]])     #係數矩陣
    #Center=[0.0,0.0]
    Ccode=0   #Confirmation_Code
    Sendcode=0
    
    #鏡頭定位
    cx,cy,ux,uy=PS.FindCenter()
    a,b,c,d=CO.getMatrix(cx,cy,ux,uy)
    
    
    
    #發送訊號啟動自走車
    
    
    #接收自走車到達終點訊號
    

    #手臂任務
    while (True):
        #輸入目標座標
        X=float(input('X='))
        Y=float(input('Y='))
        Z=float(input('Z='))
        A=float(input('A='))
        #----------------初始位置 伸到 物體上--------------
        #用目標座標計算目標姿態
        Angle=IK.InverseAlgorithm(X,Y,Z+80.0)
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

        #----------------物體上 伸到 物體 --並且夾取--------------
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
        #----------------將物體提起----------------
        #用目標座標計算目標姿態
        Angle=IK.InverseAlgorithm(X,Y,Z+80.0)
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
        #----------------將物體舉至鏡頭下----------------
        #用目標座標計算目標姿態
        Angle=IK.InverseAlgorithm(150.0,0.0,250.0)
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

        time.sleep(10)
        print("4K")
        #----------------將物體丟垃圾桶----------------
        #用目標座標計算目標姿態
        Angle=IK.InverseAlgorithm(-250.0,0.0,150.0)
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


        print("5K")
        #----------------回歸初始位置----------------
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
        
    
    
    



if __name__ == "__main__":
    main()
