import cv2
import numpy as np
import matplotlib.pyplot as plt
    
lanes = cv2.imread("clean-parking-lot.jpeg")
lanes = cv2.cvtColor(lanes, cv2.COLOR_BGR2HSV)

low_white = (0,0,150)
high_white = (255,255,255)

mask = cv2.inRange(lanes, low_white, high_white)
result = cv2.bitwise_and(lanes, lanes, mask = mask)

 
print("done")

plt.imshow(mask, cmap = "gray")
plt.show()

