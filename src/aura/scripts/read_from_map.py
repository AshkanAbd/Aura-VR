#!/usr/bin/env python

import math
import cv2 as cv
import numpy as np

img = cv.imread('/home/ashkan/Desktop/example.png')
mask_img = np.zeros((img.shape), np.uint8)
mask_img = cv.cvtColor(mask_img, cv.COLOR_BGR2GRAY)
mask_img = cv.ellipse(mask_img, (500, 500), (25,25), -22, 0, 45, 255, -1)
res = cv.bitwise_and(img, img, mask=mask_img)
resbgr = res.copy()
res = cv.cvtColor(res, cv.COLOR_BGR2GRAY)
victim = np.array([[200, 200],
                   [200, 0],
                   [200, 200]], np.uint8)
res1 = cv.matchTemplate(res, victim, cv.TM_CCOEFF)
_, _, min_loc, max_loc = cv.minMaxLoc(res1)
res2 = cv.rectangle(resbgr, max_loc, (max_loc[0] + 10, max_loc[1] + 10), (255, 0, 0), 1)
cv.imshow('a', img)
cv.imshow('b', mask_img)
cv.imshow('c', res)
cv.imshow('d', res1)
cv.imshow('e', res2)
cv.waitKey(0)
