import cv2
import numpy as np
from PIL import Image

def FindContour():
    cap = cv2.VideoCapture(0)
    xs=None
    ys=None
    A=None
    number_of_cnts=0
    counter=0

    while True:
        
        
        #讀取鏡頭影像
        ret, frame = cap.read() 
        
        #將影像轉為灰階
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        #影像模糊化
        blurred = cv2.GaussianBlur(gray,(9, 9 ), 0)
        blurred1 = cv2.GaussianBlur(blurred,(13, 13), 0)
         #blurred2 = cv2.GaussianBlur(blurred1,(5, 5), 0)
         #frame = cv2.GaussianBlur(frame,(9, 9), 0)

        
        ret , thresh = cv2.threshold(blurred1,135,255,cv2.THRESH_BINARY_INV)

        #尋找所有輪廓
        (cnts, _ ) = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #複製一個影像檔
        frame1 = frame.copy()

        
        #解析每一個輪廓
        for cnt in cnts:
            #取得一個輪廓的面積
             area = cv2.contourArea(cnt)

            #判斷面積大小 符合才選取
             if (area >= 20000) & (area <=30000):
                #取最小方形
                rect = cv2.minAreaRect(cnt)
                
                #取得方形中點座標
                x, y = rect[0]
                
                
                #在此點畫上圓點
                cv2.circle(frame1, (int(x), int(y)), 3, (0, 255, 0), 5)

                #取得此方形之長寬
                width, height = rect[1]

                #取得此方形之傾角
                angle = rect[2]
                
                if (x>=xs):
                    xs=x
                    ys=y
                    A=angle
                
                #沿著此方形畫上方形線
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame1, [box], 0, (0, 0, 255), 2)
                
                #印出方框資訊
                print ('width=', width, 'height=', height, 'x=', x, 'y=', y, 'angle=', angle)
                print ('area=' , rect)
            

                #開一個視窗顯示處理完的影像
                cv2.imshow('CONTOUR', frame1)
                #cv2.imshow('gray', gray)
                #cv2.imshow('blurred', blurred)
                #cv2.imshow('thresh', thresh)
                #cv2.imshow('frame1', frame1)
            
                

        if ((xs!=None)&(ys!=None)):
            break
        
        if cv2.waitKey(5) & 0xFF == 27:
            break

    cv2.destroyAllWindows()
    
    return xs,ys,A
