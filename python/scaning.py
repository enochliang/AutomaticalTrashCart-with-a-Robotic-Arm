import cv2
import numpy as np
from PIL import Image

cap = cv2.VideoCapture(0)

while True:
        ret, frame = cap.read() 

        
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        blurred = cv2.GaussianBlur(gray,(9, 9 ), 0)
        blurred1 = cv2.GaussianBlur(blurred,(13, 13), 0)
        #blurred2 = cv2.GaussianBlur(blurred1,(5, 5), 0)
        #frame = cv2.GaussianBlur(frame,(9, 9), 0)

        ret , thresh = cv2.threshold(blurred1,135,255,cv2.THRESH_BINARY_INV)

        (cnts, _ ) = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        

        frame1 = frame.copy()

        
        for cnt in cnts:
             area = cv2.contourArea(cnt)

             if area >= 20000:
                rect = cv2.minAreaRect(cnt)

                x, y = rect[0]
                cv2.circle(frame1, (int(x), int(y)), 3, (0, 255, 0), 5)

                width, height = rect[1]

                angle = rect[2]

                box = cv2.boxPoints(rect)

                box = np.int0(box)

                cv2.drawContours(frame1, [box], 0, (0, 0, 255), 2)

                print ('width=', width, 'height=', height, 'x=', x, 'y=', y, 'angle=', angle)
                print ('area=' , rect)
            

        
                cv2.imshow('CONTOUR', frame1)
                #cv2.imshow('gray', gray)
                #cv2.imshow('blurred', blurred)
                #cv2.imshow('thresh', thresh)
                #cv2.imshow('frame1', frame1)

        
        if cv2.waitKey(5) & 0xFF == 27:
            break

         
cv2.destroyAllWindows()

