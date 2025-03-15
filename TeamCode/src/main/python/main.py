import cv2
import numpy as np

kSize = 13
blur = 5
imgGain = 1.3

def getElements(inImg: cv2.Mat, kr: float, kg: float, kb: float, threshold: float):
    b, g, r = cv2.split(inImg)

    print(b.dtype)

    def mult(mat: cv2.Mat, scale: float):
        return mat * np.array([scale])

    combinedRGB = mult(r, kr) + mult(g, kg) + mult(b, kb)

    subsR = cv2.subtract(combinedRGB, mult(r, 1.0 - kr))
    subsG = cv2.subtract(combinedRGB, mult(g, 1.0 - kg))
    subsB = cv2.subtract(combinedRGB, mult(b, 1.0 - kb))

    inRangedR = cv2.inRange(subsR, np.array([threshold]), np.array([255.0]))
    inRangedG = cv2.inRange(subsG, np.array([threshold]), np.array([255.0]))
    inRangedB = cv2.inRange(subsB, np.array([threshold]), np.array([255.0]))

    combined = cv2.bitwise_and(inRangedR, cv2.bitwise_and(inRangedG, inRangedB))

    combined = cv2.erode(combined, cv2.getStructuringElement(cv2.MORPH_RECT, (kSize, kSize)))
    combined = cv2.dilate(combined, cv2.getStructuringElement(cv2.MORPH_RECT, (kSize, kSize)))

    contours, _ = cv2.findContours(combined, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return contours

def drawCountours(inImg, contours, r, g, b):
    for i in contours:
        rect = cv2.minAreaRect(i)
        box = cv2.boxPoints(rect)
        box = np.int16(box)
        cv2.line(inImg, box[0], box[1], (b, g, r), 3)
        cv2.line(inImg, box[1], box[2], (b, g, r), 3)
        cv2.line(inImg, box[2], box[3], (b, g, r), 3)
        cv2.line(inImg, box[3], box[0], (b, g, r), 3)

img = cv2.imread('img9.jpg')

height, width, _ = img.shape

img = cv2.resize(img, (int(width * imgGain), int(height * imgGain)))
drawImg = img.copy()

img = cv2.normalize(img, np.array([]), alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)

img = cv2.blur(img, (blur, blur))

drawCountours(drawImg, getElements(img, 1.0, 0.0, 0.0, 85.0), 200, 0, 0)
drawCountours(drawImg, getElements(img, 0.0, 0.0, 1.0, 85.0), 0, 0, 200)
drawCountours(drawImg, getElements(img, 0.5, 0.5, 0.0, 85.0), 200, 200, 0)

cv2.imshow('open cv', drawImg)
cv2.waitKey(0)