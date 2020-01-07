import numpy as np
import cv2
#https://stackoverflow.com/questions/57008502/is-there-a-way-to-use-video-capture-read-in-opencv-without-it-taking-so-long

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, 0)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()