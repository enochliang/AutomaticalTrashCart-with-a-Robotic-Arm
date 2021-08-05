import cv2
import numpy as np

def FindCenter():
    xRED=None
    yRED=None
    xGREEN=None
    yGREEN=None
    cap = cv2.VideoCapture(0)
    while(1):
        #讀取影像
        
        ret, frame = cap.read() 

        #取得影像Frame_SIZE
        #width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        #height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    
        #影像處理
        blurred = cv2.GaussianBlur(frame,(9, 9 ), 0)
        blurred1 = cv2.GaussianBlur(blurred,(11, 11 ), 0)

        #紅色遮罩
        hsv = cv2.cvtColor(blurred1, cv2.COLOR_BGR2HSV)

        erode_hsv = cv2.erode(hsv, None, iterations=2)
        
        lower_red = np.array([-3, 50, 40])
        upper_red = np.array([10, 255, 255])
         
        red_mask = cv2.inRange(erode_hsv, lower_red, upper_red)

        #雜訊處理
        #kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(50, 50))
        #opened = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel) 
        #closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        
        #取得所有輪廓資訊
        Rcnts = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        #解析每一個輪廓
        for cnt in Rcnts:
           
            area = cv2.contourArea(cnt)

            if  (area >= 25) & (area <=200):   #判斷面積大小
                rect = cv2.minAreaRect(cnt)      #框住物體的最小矩形

                xRED, yRED = rect[0]          ###########矩形中心座標
                xRED=int(xRED)
                yRED=int(yRED)
                cv2.circle(frame, (int(xRED), int(yRED)), 1, (0, 255, 0), 5)

                box = cv2.boxPoints(rect)

                box = np.int0(box)

                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

                #print ('xRED=', xRED, 'yRED=', yRED)
                

        #cv2.imshow('CONTOUR', frame)
    
        #綠色遮罩 參數代測
        #hsv = cv2.cvtColor(blurred1, cv2.COLOR_BGR2HSV)

        #erode_hsv = cv2.erode(hsv, None, iterations=2)
        
        lower_green = np.array([70, 80, 30])
        upper_green = np.array([110, 255, 255])
     
        green_mask = cv2.inRange(erode_hsv, lower_green, upper_green)

        #雜訊處理 參數代測
        
        #opened = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel) 
        #closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)

        #取得所有輪廓資訊
        Gcnts = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
        for cnt in Gcnts:
           
            area = cv2.contourArea(cnt)

            if  (area >= 25) & (area <=200):   #判斷面積大小
                rect = cv2.minAreaRect(cnt)      #框住物體的最小矩形

                xGREEN, yGREEN = rect[0]          ###########矩形中心座標
                xGREEN=int(xGREEN)
                yGREEN=int(yGREEN)
                cv2.circle(frame, (int(xGREEN), int(yGREEN)), 1, (0, 255, 0), 5)

                box = cv2.boxPoints(rect)

                box = np.int0(box)

                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

                #print ('xGREEN=', xGREEN, 'yGREEN=', yGREEN)

        cv2.imshow('CONTOUR', frame)


        if (xRED!=None)and(yRED!=None)and(xGREEN!=None)and(yGREEN!=None):
            break
        if cv2.waitKey(3) & 0xFF == 27:
            break

##    print(xRED)
##    print(yRED)
##    print(xGREEN)
##    print(yGREEN)
    cv2.destroyAllWindows()
    #print((480-yRED),(640-xRED),(480-yGREEN),(640-xGREEN))
    #return xRED,(480-yRED),xGREEN,(480-yGREEN)
    return (480-yRED),(640-xRED),(480-yGREEN),(640-xGREEN)





#FindCenter()
