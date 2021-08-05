import cv2
import numpy as np
import time
#需下載外部模塊
from PIL import Image


def FindContour():
    #開啟攝像頭
    cap = cv2.VideoCapture(0)

    #儲存上一張圖最右邊的輪廓的資訊
    XS=None
    YS=None
    AS=None
    counter=0
    

    #進入讀取影像迴圈
    while True:
        
        #宣告X,Y,Angle變數
        xs=0
        ys=None
        A=None



        #-------------影像處理---------------
        #讀取鏡頭影像
        ret, frame = cap.read() 
        
        #將影像轉為灰階
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #影像模糊化
        blurred = cv2.GaussianBlur(gray,(9, 9), 0)
        blurred1 = cv2.GaussianBlur(blurred,(11, 11), 0)
        
        #影像二值化
        ret ,thresh = cv2.threshold(blurred1,110,255,cv2.THRESH_BINARY_INV)

        #binaryIMG = cv2.Canny(blurred1, 20, 160)
        #kernel = np.ones((5,5),np.uint8)
        #dilation = cv2.dilate(thresh,kernel,iterations = 1)

        #尋找所有輪廓
        (cnts, _ ) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #複製一個影像檔
        frame1 = frame.copy()


        
        #--------------------解析此照片的每一個輪廓並繪圖-----------------
        for cnt in cnts:
            #取得一個輪廓的面積
             area = cv2.contourArea(cnt)

            #判斷面積大小 符合才選取
             if (area >= 5000) & (area <=15000):

                
                
                #取最小方形
                rect = cv2.minAreaRect(cnt)
                
                #取得方形中點座標
                x, y = rect[0]

                if(y>400):
                    continue
                #在此點畫上圓點
                cv2.circle(frame1, (int(x), int(y)), 3, (0, 255, 0), 5)

                #取得此方形之長寬
                width, height = rect[1]

                #取得此方形之傾角
                angle = rect[2]

                #讀取最右邊的物體
                if (x>=xs):
                    xs=x
                    ys=y
                    A=angle
                    if (width>height):
                        A=-A
                    else:
                        A=-(A+90.0)
                #print(xs)
                #沿著此方形畫上方形線
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame1, [box], 0, (0, 0, 255), 2)
            



        #------------------------顯示結果--------------------------
        #開一個視窗顯示處理完的影像
        cv2.imshow('CONTOUR', frame1)
        cv2.imshow('thresh', thresh)
        





        #如果有輪廓資訊就進入下一層判斷
        if(ys!=None):
            
            #如果上次的資料有東西就進入下一層判斷
            if(YS!=None):
                #如果本次讀到的資訊跟上次一樣就進入下一層判斷
                if((abs(xs-XS)<10)&(abs(ys-YS)<10)&(abs(A-AS)<2)):
                    #有兩種情況
                    #已經一樣一次
                    if(counter==1):
                        break
                    #第一次一樣
                    elif(counter==0):
                        counter=0
                        #counter=1並計時
                        counter+=1
                        time.sleep(0.5)
                        continue
                    
            counter=0
            
        #將本次資訊存為下次的上一張資訊
        XS=xs
        YS=ys
        AS=A



        #按鍵強迫終止程式
        if cv2.waitKey(5) & 0xFF == 27:
            break

    #列印資料
    #print(xs)
    print((480.0-ys),(640.0-xs),A)
    #print((480.0-YS),(640.0-XS),A)

    cv2.destroyAllWindows()
    
    return (480.0-ys),(640-xs),A



#FindContour()
