import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0)
ret, frame1 = cap.read()
frame1 = cv2.resize(frame1, (320,240))
prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[...,1] = 255
while(1):
    ret, frame2 = cap.read()
    frame2 = cv2.resize(frame2, (320,240))
    next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
    start = time.time()
    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 1, 15, 1, 5, 1.2, 0)
    print time.time() - start
    mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
    hsv[...,0] = ang*180/np.pi/2
    hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
    bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    cv2.imshow('frame2',bgr)
    cv2.waitKey(1)
    # k = cv2.waitKey(30) & 0xff
    # if k == 27:
    #     break
    # elif k == ord('s'):
    #     cv2.imwrite('opticalfb.png',frame2)
    #     cv2.imwrite('opticalhsv.png',bgr)
    prvs = next
cap.release()
cv2.destroyAllWindows()