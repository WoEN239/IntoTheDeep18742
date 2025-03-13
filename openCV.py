import cv2
import numpy as np

kSize = 9
blur = 10
imgGain = 0.8

img = cv2.imread('img1.jpg')

height, width, _ = img.shape

img = cv2.resize(img, (int(width * imgGain), int(height * imgGain)))
drawImg = img.copy()

img = cv2.normalize(img, None, alpha=0,beta=255,norm_type=cv2.NORM_MINMAX)

img = cv2.blur(img, (blur, blur))
b, g, r = cv2.split(img)

b //= 2
g //= 2
r //= 2
r+=126

inRangedRG = cv2.inRange(cv2.subtract(r, g), np.array([152]), np.array([255]))
inRangedRB = cv2.inRange(cv2.subtract(r, b), np.array([161]), np.array([255]))
inRangedG = cv2.inRange(g, np.array([0]), np.array([55]))

combinedR = cv2.bitwise_and(cv2.bitwise_and(inRangedRG, inRangedRB), inRangedG)

combinedR = cv2.erode(combinedR, cv2.getStructuringElement(cv2.MORPH_RECT, (kSize, kSize)))
combinedR = cv2.dilate(combinedR, cv2.getStructuringElement(cv2.MORPH_RECT, (kSize, kSize)))

contours, _ = cv2.findContours(combinedR.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# if len(contours) > 0:
#     cv2.drawContours(drawImg, contours, -1, (0, 0, 255), 3)

for i in contours:
    rect = cv2.minAreaRect(i)
    box = cv2.boxPoints(rect)
    box = np.int16(box)
    cv2.line(drawImg, box[0], box[1], (0, 0, 255), 3)
    cv2.line(drawImg, box[1], box[2], (0, 0, 255), 3)
    cv2.line(drawImg, box[2], box[3], (0, 0, 255), 3)
    cv2.line(drawImg, box[3], box[0], (0, 0, 255), 3)

cv2.imshow('open cv', drawImg)
cv2.imshow("binaryRG", inRangedRG)
cv2.imshow("binaryRB", inRangedRB)
cv2.imshow("binaryG", inRangedG)
cv2.waitKey(0)