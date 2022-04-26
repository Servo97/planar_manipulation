import numpy as np
import cv2
import matplotlib.pyplot as plt
import os
from glob import glob


for i in ['before','after']:
    list_of_files = glob(f'./data/gray_{i}/*') # * means all if need specific format then *.csv
    for f in list_of_files:
        img = cv2.imread(latest_file,0)
        img = cv2.resize(img, (96,64), interpolation = cv2.INTER_AREA)
        img = img[:,10:]

        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
        _, out = cv2.threshold(img,110,255,cv2.THRESH_BINARY_INV)
        out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel)

        temp = out.copy()
        temp[temp>100] = 1
        print(temp)
        print(np.sum(temp))


        cv2.imwrite(latest_file.replace('gray','binary'), out)