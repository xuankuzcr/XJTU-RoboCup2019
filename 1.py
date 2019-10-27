import cv2
import numpy as np
video=cv2.VideoCapture('../10.9识别/培训任务1视频.mp4')
fps = video.get(cv2.CAP_PROP_FPS)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
lg=np.array([33,70,82])
ug=np.array([78,255,255])
font = cv2.FONT_HERSHEY_SIMPLEX 


try:
    while True: 
        ret, frame =video.read()
        frame1= cv2.GaussianBlur(frame, (21, 21),0)
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,lg, ug)
        res = cv2.dilate(mask,kernel, iterations=3)
        contours, hierarchy = cv2.findContours(res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for con in contours:       
            x, y, w, h = cv2.boundingRect(con)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0,255), 3)
            cv2.putText(frame, 'Here', (int(x+w/2),int( y+h+50)), font, 2, (0, 0, 255), 2)
        cv2.imshow('video',frame)
        
        if cv2.waitKey(int(1000//fps)) & 0xff == ord('q'):
            break
except:
    pass

#contours, hierarchy = cv2.findContours(res, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
 #res=_.copy()


#size=(int(video.get(cv2.CAP_PROP_FRAME_WIDTH)),int(viedo.get(cv2.CAP_PROP_FRAME_HEIGHT)))
#print('size:'+repr(size))




'''
for (x,y,w,h) in faces:
        img = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)#在每一帧中画出人脸的矩阵


        roi_gray = gray[y:y+h,x:x+w]
        eyes = eye_cascade.detectMultiScale(roi_gray,1.03,5,0,(40,40))#在人脸的范围内寻找眼睛

        for (ex,ey,ew,eh) in eyes:
            cv2.rectangle(img,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
'''
#画矩形
