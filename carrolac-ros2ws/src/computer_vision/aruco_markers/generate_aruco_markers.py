#!/usr/bin/env python3

import cv2

arudict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
sidePixels = 400
borderBits = 1
whiteMargin = int(sidePixels / (6 + 2 * borderBits))

for i in range(arudict.bytesList.shape[0]):
    istr = "{:0>4}".format(i)
    print("Processing Image number " + istr)
    img = cv2.aruco.generateImageMarker(arudict, id=i, sidePixels=400, borderBits=1)
    img = cv2.copyMakeBorder(img, whiteMargin, whiteMargin, whiteMargin, whiteMargin,
                             cv2.BORDER_CONSTANT, value=[255, 255, 255])
    cv2.imwrite("markers/AU" + istr + ".png", img)

# input()
