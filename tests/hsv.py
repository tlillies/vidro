import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow('res')

cv2.createTrackbar('H_lower','res',157,255, nothing)
cv2.createTrackbar('S_lower','res',149,255,nothing)
cv2.createTrackbar('V_lower','res',67,255,nothing)
	
cv2.createTrackbar('H_upper','res',186,255,nothing)
cv2.createTrackbar('S_upper','res',255,255,nothing)
cv2.createTrackbar('V_upper','res',255,255,nothing)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    H_lower = cv2.getTrackbarPos('H_lower','res',)
    S_lower = cv2.getTrackbarPos('S_lower','res')
    V_lower = cv2.getTrackbarPos('V_lower','res')
    
    H_upper = cv2.getTrackbarPos('H_upper','res')
    S_upper = cv2.getTrackbarPos('S_upper','res')
    V_upper = cv2.getTrackbarPos('V_upper','res')

    lower = np.array([H_lower, S_lower, V_lower], dtype=np.uint8)
    upper = np.array([H_upper,S_upper,V_upper], dtype=np.uint8)

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)
    
    kernel_open = np.ones((2,2),np.uint8)
    kernel_close = np.ones((3,3),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
        
    #contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(mask, contours, -1, (0,255,0), 3)
    
    
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    # Display the resulting frame
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
