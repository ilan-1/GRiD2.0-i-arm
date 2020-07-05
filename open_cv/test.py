import numpy as np
import cv2

img = cv2.imread('img1.jpg',0)
img_n = cv2.resize(img, (0, 0),fx = 0.1, fy = 0.1)

cv2.imshow('image',img_n)
k = cv2.waitKey(0) & 0xFF

if k == 27:
    cv2.destroyAllWindows()
